#!/usr/bin/env python3

import rospy
import tkinter as tk
import rospkg
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import sys
import os
import signal
from gazebo_msgs.srv import SpawnModel
from std_srvs.srv import Empty
import subprocess

rospy.init_node("dbot_controller_gui")
rospack = rospkg.RosPack()

# Bot Controls
door_pub = rospy.Publisher(
    "/dbot/door_position_controller/command",
    Float64,
    queue_size=10
)


pusher_pub = rospy.Publisher(
    "/dbot/pusher_position_controller/command",
    Float64,
    queue_size=10
)

FLOOR_HEIGHT = 4.0
DOOR_OPEN = 0.51
DOOR_CLOSED = -1.57075
PUSHER_OUT = 0.24
PUSHER_IN = 0.0
TOLERANCE_DOOR = 0.05
TOLERANCE_PUSHER = 0.01 


def open_bot_door():
    rospy.loginfo("Opening Bot Door")
    door_pub.publish(DOOR_OPEN)


def close_bot_door():
    rospy.loginfo("Closing Bot Door")
    door_pub.publish(DOOR_CLOSED)


def pusher_out():
    rospy.loginfo("Pushing Package Out")
    pusher_pub.publish(PUSHER_OUT)


def pusher_in():
    rospy.loginfo("Retracting Pusher")
    pusher_pub.publish(PUSHER_IN)


def deliver_package():
    global DOOR_CLOSED, DOOR_OPEN, PUSHER_IN, PUSHER_IN, TOLERANCE_DOOR,TOLERANCE_PUSHER
    DOOR_STATE = None
    PUSHER_STATE = None

    def check_door():
        msg = rospy.wait_for_message('/dbot/joint_states', JointState)
        DOOR_STATE = msg.position[0]
        return DOOR_STATE

    def check_pusher():
        msg = rospy.wait_for_message('/dbot/joint_states', JointState)
        PUSHER_STATE = msg.position[1]
        return PUSHER_STATE
    
    while DOOR_STATE < DOOR_OPEN - TOLERANCE_DOOR:
        DOOR_STATE = check_door()
        open_bot_door()

    while PUSHER_STATE < PUSHER_OUT - TOLERANCE_PUSHER:
        PUSHER_STATE = check_pusher()
        pusher_out()

    while PUSHER_STATE > PUSHER_IN + TOLERANCE_PUSHER:
        PUSHER_STATE = check_pusher()
        pusher_in()

    while DOOR_STATE > DOOR_CLOSED + TOLERANCE_DOOR:
        DOOR_STATE = check_door()
        close_bot_door()

    rospy.loginfo("Package Drop Complete")
     

# Elevator Controls

elevator_door = {
    "CLOSED": 0,
    "OPEN": 1,
}

floors = {
    0 : "lobby",
    1 : "first",
    2 : "second",
}

floor_door_left_pubs = []
floor_door_right_pubs = []

for i in range(len(floors.keys())):
    floor_door_left_pubs.append(
        rospy.Publisher(
        '/elevator/' + str(floors[i]) + '_door_left_controller/command',
        Float64,
        queue_size=10)
    )

    floor_door_right_pubs.append(rospy.Publisher(
        '/elevator/' + str(floors[i]) + '_door_right_controller/command',
        Float64,
        queue_size=10)
    )

floor_pub = rospy.Publisher(
    '/elevator/floor_position_controller/command',
    Float64,
    queue_size=10
)

pub_car_left = rospy.Publisher(
    '/elevator/car_door_left_controller/command',
    Float64,
    queue_size=10
)
    
pub_car_right = rospy.Publisher(
    '/elevator/car_door_right_controller/command',
    Float64,
    queue_size=10
)

ELEVATOR_DOOR_OPEN = 0.5
FLOOR = 0
ELEVATOR_DOOR = elevator_door["CLOSED"]


def go_to_floor(floor):
    global FLOOR, ELEVATOR_DOOR, FLOOR_HEIGHT
    rospy.loginfo("Going to " + str(floor) + " floor")
    FLOOR = int(floor)
    if ELEVATOR_DOOR != elevator_door["CLOSED"]:
        close_elevator_doors()

    floor_pub.publish(FLOOR_HEIGHT * floor)


def open_elevator_doors():
    global ELEVATOR_DOOR

    rospy.loginfo("OPENING DOORS" + " FLOOR " + str(FLOOR))

    
    floor_door_right_pubs[FLOOR].publish(ELEVATOR_DOOR_OPEN)
    floor_door_left_pubs[FLOOR].publish(-ELEVATOR_DOOR_OPEN)

    pub_car_right.publish(ELEVATOR_DOOR_OPEN)
    pub_car_left.publish(-ELEVATOR_DOOR_OPEN)

    ELEVATOR_DOOR = elevator_door["OPEN"]


def close_elevator_doors():
    global ELEVATOR_DOOR
    rospy.loginfo("CLOSING DOORS")

    floor_door_right_pubs[FLOOR].publish(0)
    floor_door_left_pubs[FLOOR].publish(0)

    pub_car_right.publish(0)
    pub_car_left.publish(0)
    
    ELEVATOR_DOOR = elevator_door["CLOSED"]

# Floor controller


MAPS_DIR = os.path.join(rospack.get_path('deliverybot'), "maps", "building")
floors = {
    0 : "lobby",
    1 : "first",
    2 : "second",
}

term = None

def change_map(floor):
    global term
    if term is not None:
        term.send_signal(signal.SIGINT)
    print("Changing map to " + floors[floor])
    term = subprocess.Popen(["rosrun", "map_server", "map_server", str(os.path.join(MAPS_DIR, floors[floor] + ".yaml"))])
    print("Map to " + floors[floor])

# Package Model Spawner
spawn_package_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)


def spawn_package(package_size):
    models_path = os.path.join(rospack.get_path('deliverybot_gazebo'), 'models', 'payloads')
    rospy.loginfo("Package Spawned")
    pose = Pose()
    pose.position.z = 0.15
    spawn_package_client(
        model_name='payload',
        model_xml=open(os.path.join(models_path, package_size + ".sdf"), 'r').read(),
        robot_namespace='',
        initial_pose = pose,
        reference_frame = "dbot::base_link"
    )


# auxiliary commands
clear_costmaps_client = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

def clear_costmaps():
    rospy.loginfo("Clearing Costmaps")
    clear_costmaps_client()


class Application(tk.Frame):

    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.master = master
        self.pack()
        self.create_widgets()


    def create_widgets(self):
        self.create_bot_widgets()
        self.create_elevator_widgets()
        self.create_map_widgets()
        self.create_prop_widgets()
        self.create_auxiliary_widgets()


    def create_bot_widgets(self):
        dbot_control_frame = tk.LabelFrame(self, text="DeliveryBot Controls")
        dbot_control_frame.grid_rowconfigure(0, weight=1)
        dbot_control_frame.grid_columnconfigure(0, weight=1)

        dbot_control_frame.pack(fill='x', pady=5)
        self.btn_open_bot_door = tk.Button(dbot_control_frame, text="Open Door", command=open_bot_door)
        self.btn_open_bot_door.grid(row=0,column=0, sticky="ew")

        self.btn_close_bot_door = tk.Button(dbot_control_frame, text="Close Door", command=close_bot_door)
        self.btn_close_bot_door.grid(row=1,column=0, sticky="ew")

        self.btn_pusher_out = tk.Button(dbot_control_frame, text="Pusher Out", command=pusher_out)
        self.btn_pusher_out.grid(row=0,column=1, sticky="ew")

        self.btn_pusher_in = tk.Button(dbot_control_frame, text="Pusher In", command=pusher_in)
        self.btn_pusher_in.grid(row=1,column=1,sticky="ew")

        self.btn_deliver = tk.Button(dbot_control_frame, text="Deliver Package", command=deliver_package)
        self.btn_deliver.grid(row=0, column=2, rowspan=2, sticky="news")


    def create_elevator_widgets(self):
        elevator_control_frame = tk.LabelFrame(self, text="Elevator Controls")
        elevator_control_frame.grid_rowconfigure(0, weight=1)
        elevator_control_frame.grid_columnconfigure(0, weight=1)
        elevator_control_frame.pack(fill="x")
        selected_floor = tk.IntVar()

        self.floor_label = tk.Label(elevator_control_frame, text="Floor")
        self.floor_label.grid(row=0, column=0, sticky="ew")

        self.dropdown_floors = tk.OptionMenu(elevator_control_frame, selected_floor, 0, 1, 2, command=go_to_floor)
        self.dropdown_floors.grid(row=1, column=0, sticky="ew")

        self.btn_open_elevator_doors = tk.Button(elevator_control_frame, text="Open Doors", command = open_elevator_doors)
        self.btn_open_elevator_doors.grid(row=0, column=1, sticky="ew")
        
        self.btn_close_elevator_doors = tk.Button(elevator_control_frame, text="Close Doors", command = close_elevator_doors)
        self.btn_close_elevator_doors.grid(row=1, column=1, sticky="ew")        


    def create_map_widgets(self):
        map_widgets_frame = tk.LabelFrame(self, text="Map Control")
        map_widgets_frame.pack(fill='x')
        map_widgets_frame.grid_rowconfigure(0, weight=1)
        map_widgets_frame.grid_columnconfigure(0, weight=1)
        
        selected_floor = tk.IntVar()
        self.floor_label = tk.Label(map_widgets_frame, text="Map floor")
        self.floor_dropdown = tk.OptionMenu(map_widgets_frame, selected_floor, 0, 1, 2, command=change_map)

        self.floor_label.grid(row=0, column=0, sticky="ew")
        self.floor_dropdown.grid(row=0, column=1, sticky="ew")


    def create_prop_widgets(self):
        prop_widgets_frame = tk.LabelFrame(self, text="Packages")
        prop_widgets_frame.pack(fill='x')
        prop_widgets_frame.grid_columnconfigure(0, weight=1, uniform="hello")

        self.small_btn = tk.Button(prop_widgets_frame, text="Small", command=lambda: spawn_package("small_payload"))
        self.medium_btn = tk.Button(prop_widgets_frame, text="Medium")
        self.large_btn = tk.Button(prop_widgets_frame, text="Large")

        self.large_btn.grid(row=0, column=0, sticky="ew")
        self.medium_btn.grid(row=0, column=1, sticky="ew")
        self.small_btn.grid(row=0, column=2, sticky="ew")


    def create_auxiliary_widgets(self):
        aux_widgets_frame = tk.LabelFrame(self, text="Auxiliary Commands")
        aux_widgets_frame.pack(fill="x")
        aux_widgets_frame.grid_columnconfigure(0, weight=1)
        
        self.clear_costmaps_btn = tk.Button(aux_widgets_frame, text="Clear Costmaps", command = clear_costmaps)
        self.clear_costmaps_btn.grid(row=0, column=0, sticky="ew")


if __name__ == "__main__":
    root = tk.Tk()
    root.title("DeliveryBot Controller")
    root.resizable(False, False)
    change_map(0)
    app = Application(master=root)
    app.mainloop()


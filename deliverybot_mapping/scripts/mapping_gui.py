#!/usr/bin/env python

import Tkinter as tk
import rospkg
import roslaunch
import rospy
import os
import geometry_msgs.msg
import tf2_ros
import tf_conversions
import yaml
import subprocess

    
class MappingGUI(tk.Frame):


    def __init__(self, master=None):
        tk.Frame.__init__(self, master=master)
        self.master = master
        self.BUILDING_NAME = None
        self.map_server_node = None
        self.rospack = rospkg.RosPack()
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.PKG_DIR = self.rospack.get_path('deliverybot_mapping')
        self.MAPS_DIR = os.path.join(self.PKG_DIR, 'maps')
        self.LAUNCH_DIR = os.path.join(self.PKG_DIR, 'launch')

        self.map = {
            "building": {
                "name": None,
                "scheme": None,
                "floors": None,
            },
            "lobby":{
                "pickup": None,
                "drop": None,
            },
            "elevator":{
                "in": None,
                "out": None,
            },
            "rooms":{
                                
            }
        }
        self.pack(expand=True, fill="both")
        self.create_widgets()


    def save_yaml(self):
        with open(os.path.join(self.MAPS_DIR, self.BUILDING_NAME, 'building.yaml'), 'w') as f:
            yaml.dump(self.map, f)        


    def connect_map(self, building_name):
        self.BUILDING_NAME = building_name
        self.map["building"]["name"] = building_name
        if os.path.isdir(os.path.join(self.MAPS_DIR, building_name)):
            with open(os.path.join(self.MAPS_DIR, building_name, 'building.yaml'), 'r') as f:
                self.map = yaml.safe_load(f)
            rospy.loginfo("Building " + building_name + " loaded.")                                                
        else:
            os.mkdir(os.path.join(self.MAPS_DIR, building_name))
            with open(os.path.join(self.MAPS_DIR, building_name, 'building.yaml'), 'w') as f:
                yaml.dump(self.map, f)
            rospy.loginfo("Building " + building_name + " created.")                


    def start_map_server(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.map_server_node = roslaunch.parent.ROSLaunchParent(
            uuid,
            [
                os.path.join(self.LAUNCH_DIR, "map_server.launch")
            ]
        )
        self.map_server_node.start()
        rospy.loginfo("Map Server started")


    def kill_map_server(self):
        self.map_server_node.shutdown()
        self.map_server_node = None
        rospy.loginfo("Map Server shutdown")


    def save_map_lobby(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [
            os.path.join(self.LAUNCH_DIR, "map_saver_lobby.launch"),
            str("location:=" + os.path.join(self.MAPS_DIR, self.BUILDING_NAME))
            ]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [
            (
                roslaunch.rlutil.resolve_launch_arguments(cli_args)[0],
                roslaunch_args
            )
        ]
        map_saver_lobby = roslaunch.parent.ROSLaunchParent(
            uuid,
            roslaunch_file
        )
        map_saver_lobby.start()
        rospy.loginfo("Lobby map saved.")


    def save_map_floor(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [
            os.path.join(self.LAUNCH_DIR, "map_saver_floor.launch"),
            str("location:=" + os.path.join(self.MAPS_DIR, self.BUILDING_NAME))
            ]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [
            (
                roslaunch.rlutil.resolve_launch_arguments(cli_args)[0],
                roslaunch_args
            )
        ]
        map_saver_lobby = roslaunch.parent.ROSLaunchParent(
            uuid,
            roslaunch_file
        )
        map_saver_lobby.start()
        rospy.loginfo("Floor map saved.")


    def mark_pickup(self):
        print("Pickup Marked")
        trans = self.tfBuffer.lookup_transform(
            'map',
            'dbot/base_link',
            rospy.Time()
        )
        self.map["lobby"]["pickup"] = {
            "x": trans.transform.translation.x,
            "y": trans.transform.translation.y,
            "a": tf_conversions.transformations.euler_from_quaternion(
                [
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
            )[2]
        }
        self.save_yaml()


    def mark_drop(self):
        print("Drop Marked")
        trans = self.tfBuffer.lookup_transform(
            'map',
            'dbot/base_link',
            rospy.Time()
        )
        self.map["lobby"]["drop"] = {
            "x": trans.transform.translation.x,
            "y": trans.transform.translation.y,
            "a": tf_conversions.transformations.euler_from_quaternion(
                [
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
            )[2]
        }
        self.save_yaml()


    def mark_elevator_in(self):
        print("Elevator In Marked")
        trans = self.tfBuffer.lookup_transform(
            'map',
            'dbot/base_link',
            rospy.Time()
        )
        self.map["elevator"]["in"] = {
            "x": trans.transform.translation.x,
            "y": trans.transform.translation.y,
            "a": tf_conversions.transformations.euler_from_quaternion(
                [
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
            )[2]
        }
        self.save_yaml()


    def mark_elevator_out(self):
        print("Elevator Out Marked")
        trans = self.tfBuffer.lookup_transform(
            'map',
            'dbot/base_link',
            rospy.Time()
        )
        self.map["elevator"]["out"] = {
            "x": trans.transform.translation.x,
            "y": trans.transform.translation.y,
            "a": tf_conversions.transformations.euler_from_quaternion(
                [
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
            )[2]
        }
        self.save_yaml()


    def add_room_coods(self, room_number):
        print("Marked Room", room_number)
        trans = self.tfBuffer.lookup_transform(
            'map',
            'dbot/base_link',
            rospy.Time()
        )
        self.map["rooms"][int(room_number)] = {
            "x": trans.transform.translation.x,
            "y": trans.transform.translation.y,
            "a": tf_conversions.transformations.euler_from_quaternion(
                [
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
            )[2]
        }
        self.save_yaml()


    def add_floor_count(self, count):
        print("Floor Count Added", count)
        trans = self.tfBuffer.lookup_transform(
            'map',
            'dbot/base_link',
            rospy.Time()
        )
        self.map["building"]["floors"] = int(count)
        self.save_yaml()


    def add_naming_scheme(self, scheme):
        print("Floor Count Added", scheme)
        trans = self.tfBuffer.lookup_transform(
            'map',
            'dbot/base_link',
            rospy.Time()
        )
        self.map["building"]["scheme"] = scheme
        self.save_yaml()


    def create_widgets(self):
        self.var_room_number = tk.StringVar()
        self.var_building_name = tk.StringVar()
        self.var_wing_name = tk.StringVar()
        self.var_floors = tk.StringVar()
        self.var_scheme = tk.StringVar()

        def add_room_coods_helper():
            if (self.var_room_number.get() != ""):
                self.add_room_coods(self.var_room_number.get())
                room_entry.delete(0, tk.END)

        mapping_frame = tk.LabelFrame(self, text="Mapping")
        mapping_frame.grid_columnconfigure(0, weight=1, uniform="something")
        mapping_frame.grid_rowconfigure(0, weight=1, uniform="something")
        mapping_frame.pack(side="top", expand=True, fill="both")

        #1
        building_name_label = tk.Label(mapping_frame, text="Bldg Name")
        building_name_label.grid(column=0, row=0, sticky="ew")
        #2
        building_name_entry = tk.Entry(mapping_frame, bd=2, textvariable=self.var_building_name)
        building_name_entry.grid(column=1, row=0, sticky="ew")
        #3
        building_name_btn = tk.Button(mapping_frame, text="Create", command=lambda: self.connect_map(self.var_building_name.get()))
        building_name_btn.grid(column=2, row=0, sticky="EW")
        #4
        label_server = tk.Label(mapping_frame, text="Server")
        label_server.grid(column=0, row=1, sticky="EW")
        #5
        server_start_btn = tk.Button(mapping_frame, text="Start", command=self.start_map_server)
        server_start_btn.grid(column=1, row=1, sticky="EW")
        #6
        server_kill_btn = tk.Button(mapping_frame, text="Clear", command=self.kill_map_server)
        server_kill_btn.grid(column=2, row=1, sticky="EW")
        #7
        label_area = tk.Label(mapping_frame, text="Area")
        label_area.grid(column=0, row=2, sticky="EW")
        #8
        lobby_btn = tk.Button(mapping_frame, text="Lobby", command=self.save_map_lobby)
        lobby_btn.grid(column=1, row=2, sticky="EW")
        #9
        floor_btn = tk.Button(mapping_frame, text="Floor", command=self.save_map_floor)
        floor_btn.grid(column=2, row=2, sticky="EW")
        #10
        label_elevator = tk.Label(mapping_frame, text="Elevator")
        label_elevator.grid(column=0, row=3, sticky="EW")
        #11
        elevator_out_btn = tk.Button(mapping_frame, text="OUT", command=self.mark_elevator_out)
        elevator_out_btn.grid(column=1, row=3, sticky="EW")
        #12
        elevator_in_btn = tk.Button(mapping_frame, text="IN", command=self.mark_elevator_in)
        elevator_in_btn.grid(column=2, row=3, sticky="EW")
        #13
        label_pickup = tk.Label(mapping_frame, text="Lobby")
        label_pickup.grid(column=0, row=4, sticky="EW")
        #14
        drop_btn = tk.Button(mapping_frame, text="Mark Drop", command=self.mark_drop)
        drop_btn.grid(column=2, row=4, sticky="EW")
        #15
        pickup_btn = tk.Button(mapping_frame, text="Mark Pickup", command=self.mark_pickup)
        pickup_btn.grid(column=1, row=4, sticky="EW")
        #16
        label_room = tk.Label(mapping_frame, text="Room")
        label_room.grid(column=0, row=5, sticky="EW")
        #17
        room_entry = tk.Entry(mapping_frame, bd=2, textvariable=self.var_room_number, width=4)
        room_entry.grid(column=1, row=5, sticky="EW")
        #18
        mark_room_btn = tk.Button(mapping_frame, text="Mark", command=add_room_coods_helper)
        mark_room_btn.grid(column=2, row=5, sticky="EW")
        #19
        building_floors_label = tk.Label(mapping_frame, text="Floors")
        building_floors_label.grid(column=0, row=6, sticky="ew")
        #20
        building_floors_entry = tk.Entry(mapping_frame, bd=2, textvariable=self.var_floors)
        building_floors_entry.grid(column=1, row=6, sticky="ew")
        #21
        building_floors_btn = tk.Button(mapping_frame, text="Save", command=lambda: self.add_floor_count(self.var_floors.get()))
        building_floors_btn.grid(column=2, row=6, sticky="EW")
        #22
        building_scheme_label = tk.Label(mapping_frame, text="Scheme")
        building_scheme_label.grid(column=0, row=7, sticky="ew")
        #23
        building_scheme_entry = tk.Entry(mapping_frame, bd=2, textvariable=self.var_scheme)
        building_scheme_entry.grid(column=1, row=7, sticky="ew")
        #24
        building_scheme_btn = tk.Button(mapping_frame, text="Save", command=lambda: self.add_naming_scheme(self.var_scheme.get()))
        building_scheme_btn.grid(column=2, row=7, sticky="ew")        


if __name__ == "__main__":
    rospy.init_node("dbot_mapping", anonymous=True)
    root = tk.Tk()
    root.resizable(False, False)
    app = MappingGUI(master=root)
    tk.mainloop()

#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import sys
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse
from elevator_controls.srv import ElevatorFloorGoal, ElevatorFloorGoalResponse
global ns 
ns = rospy.get_namespace()

door = {
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
        rospy.names.ns_join(ns, str(floors[i]) + '_door_left_controller/command'),
        Float64,
        queue_size=10)
    )

    floor_door_right_pubs.append(rospy.Publisher(
        rospy.names.ns_join(ns, str(floors[i]) + '_door_right_controller/command'),
        Float64,
        queue_size=10)
    )

# Publishers
car_pos_pub = rospy.Publisher(
    rospy.names.ns_join(ns, 'floor_position_controller/command'),
    Float64,
    queue_size=10
)

pub_car_left = rospy.Publisher(
    rospy.names.ns_join(ns, 'car_door_left_controller/command'),
    Float64,
    queue_size=10
)
    
pub_car_right = rospy.Publisher(
    rospy.names.ns_join(ns, 'car_door_right_controller/command'),
    Float64,
    queue_size=10
)


DOOR_OPEN = 0.5
FLOOR = 0
DOOR = door["CLOSED"]

def go_to_floor(req):
    global FLOOR, DOOR
    goal_floor = 4.0 * req.floor
    def check_floor():
        msg = rospy.wait_for_message(rospy.names.ns_join(ns, 'joint_states'), JointState)
        FLOOR_STATE = msg.position[0]
        return FLOOR_STATE

    rospy.loginfo("Going to " + str(req.floor) + " floor")
    if DOOR != door["CLOSED"]:
        close_doors(req)
    
    while abs(check_floor() - goal_floor) > 0.001:
        car_pos_pub.publish(goal_floor)
    FLOOR = int(req.floor)        
    return ElevatorFloorGoalResponse()

def open_doors(req):
    global DOOR
    rospy.loginfo("OPENING DOORS" + " FLOOR " + str(FLOOR))    
    floor_door_right_pubs[FLOOR].publish(DOOR_OPEN)
    floor_door_left_pubs[FLOOR].publish(-DOOR_OPEN)
    pub_car_right.publish(DOOR_OPEN)
    pub_car_left.publish(-DOOR_OPEN)
    rospy.sleep(5)
    DOOR = door["OPEN"]
    return EmptyResponse()

def close_doors(req):
    global DOOR
    rospy.loginfo("CLOSING DOORS")
    floor_door_right_pubs[FLOOR].publish(0)
    floor_door_left_pubs[FLOOR].publish(0)
    pub_car_right.publish(0)
    pub_car_left.publish(0)
    rospy.sleep(5)
    DOOR = door["CLOSED"]
    return EmptyResponse()

if __name__ == "__main__":
    rospy.init_node('elevator_controller')
    rospy.Service('open_elevator_doors', Empty, open_doors)
    rospy.Service('close_elevator_doors', Empty, close_doors)
    rospy.Service('elevator_goto_floor', ElevatorFloorGoal, go_to_floor)
    rospy.spin()
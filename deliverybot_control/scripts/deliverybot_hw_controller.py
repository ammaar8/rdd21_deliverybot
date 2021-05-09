#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse

global ns 
ns = rospy.get_namespace()
print("Controller started with ns " + ns)

DOOR_OPEN = 0.51
DOOR_CLOSED = -1.57075
PUSHER_OUT = 0.24
PUSHER_IN = 0.0
TOLERANCE_DOOR = 0.05
TOLERANCE_PUSHER = 0.01 

door_pub = rospy.Publisher(
    rospy.names.ns_join(ns, "door_position_controller/command"),
    Float64,
    queue_size=10
)


pusher_pub = rospy.Publisher(
    rospy.names.ns_join(ns, "pusher_position_controller/command"),
    Float64,
    queue_size=10
)


def open_bot_door(req):
    rospy.loginfo("Opening Bot Door")
    door_pub.publish(DOOR_OPEN)
    return EmptyResponse()


def close_bot_door(req):
    rospy.loginfo("Closing Bot Door")
    door_pub.publish(DOOR_CLOSED)
    return EmptyResponse()


def pusher_out(req):
    rospy.loginfo("Pushing Package Out")
    pusher_pub.publish(PUSHER_OUT)
    return EmptyResponse()


def pusher_in(req):
    rospy.loginfo("Retracting Pusher")
    pusher_pub.publish(PUSHER_IN)
    return EmptyResponse()


def deliver_package(req):
    global DOOR_CLOSED, DOOR_OPEN, PUSHER_IN, PUSHER_IN, TOLERANCE_DOOR,TOLERANCE_PUSHER
    DOOR_STATE = None
    PUSHER_STATE = None

    def check_door():
        msg = rospy.wait_for_message(rospy.names.ns_join(ns, 'joint_states'), JointState)
        DOOR_STATE = msg.position[0]
        return DOOR_STATE

    def check_pusher():
        msg = rospy.wait_for_message(rospy.names.ns_join(ns, 'joint_states'), JointState)
        PUSHER_STATE = msg.position[1]
        return PUSHER_STATE
    
    DOOR_STATE = check_door()
    PUSHER_STATE = check_pusher()
    
    while DOOR_STATE < DOOR_OPEN - TOLERANCE_DOOR:
        DOOR_STATE = check_door()
        open_bot_door(req)

    while PUSHER_STATE < PUSHER_OUT - TOLERANCE_PUSHER:
        PUSHER_STATE = check_pusher()
        pusher_out(req)

    while PUSHER_STATE > PUSHER_IN + TOLERANCE_PUSHER:
        PUSHER_STATE = check_pusher()
        pusher_in(req)

    while DOOR_STATE > DOOR_CLOSED + TOLERANCE_DOOR:
        DOOR_STATE = check_door()
        close_bot_door(req)

    rospy.loginfo("Package Drop Complete")
    return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("deliverybot_hw_controller")    
    rospy.Service('close_bot_door', Empty, close_bot_door)
    rospy.Service('open_bot_door', Empty, open_bot_door)
    rospy.Service('set_pusher_out', Empty, pusher_out)
    rospy.Service('set_pusher_in', Empty, pusher_in)
    rospy.Service('deliver_package', Empty, deliver_package)
    rospy.spin()
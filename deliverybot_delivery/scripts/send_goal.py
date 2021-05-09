#!/usr/bin/env python

import rospy
import sys
import tf_conversions
import actionlib
from std_srvs.srv import Empty
from elevator_controls.srv import ElevatorFloorGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from deliverybot_navigation.srv import MapFilePath

global dbot_deliver, change_map, el_open_door, el_close_door, el_change_floor, clear_costmap

def movebase_client(x, y, a):
	
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.position.z = 0.0

	quat = tf_conversions.transformations.quaternion_from_euler(
		0.0,
		0.0,
		a
	)

	goal.target_pose.pose.orientation.x = quat[0]
	goal.target_pose.pose.orientation.y = quat[1]
	goal.target_pose.pose.orientation.z = quat[2]
	goal.target_pose.pose.orientation.w = quat[3]
	
	client.send_goal(goal)
	wait = client.wait_for_result()
	rospy.loginfo("Sent Goal")
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	else:
		return client.get_result()

def execute_delivery():
	change_map.call("/home/solkar/catkin_ws/src/DeliveryBot/deliverybot/deliverybot_mapping/maps/building/lobby.yaml")
	movebase_client(-0.046, -2.095, 1.483) # EL out
	el_open_door.call() # Open Doors
	rospy.sleep(5)
	movebase_client(0.0, 0.0, -1.519) # El In
	el_close_door.call() # Close Doors
	el_change_floor(2) # Chagne Floor
	rospy.sleep(20) # wait
	change_map.call("/home/solkar/catkin_ws/src/DeliveryBot/deliverybot/deliverybot_mapping/maps/building/floor.yaml") # Change map
	el_open_door.call() # Open Doors
	movebase_client(3.489, -1.518, 1.556) # go to room
	dbot_deliver.call()
	movebase_client(0.0, 0.0, -1.519) #EL In	
	el_close_door.call()# Close Doors
	el_change_floor(0) # Change floor
	rospy.sleep(20) # wait
	change_map.call("/home/solkar/catkin_ws/src/DeliveryBot/deliverybot/deliverybot_mapping/maps/building/lobby.yaml") # Change Map
	el_open_door.call() # Open Doors
	rospy.sleep(5)
	clear_costmap.call()
	movebase_client(-0.046, -2.095, -1.483) # El out
	el_close_door.call()# Close Doors
	movebase_client(0.213, -11.056, -1.545) # Go to pickup
	return True

		
if __name__ == "__main__":
	rospy.init_node("movebase_client_py")
	dbot_deliver = rospy.ServiceProxy('/dbot/deliver_package', Empty)
	change_map = rospy.ServiceProxy('/load_map', MapFilePath)
	el_open_door = rospy.ServiceProxy('/elevator/open_elevator_doors', Empty)
	el_close_door = rospy.ServiceProxy('/elevator/close_elevator_doors', Empty)
	el_change_floor = rospy.ServiceProxy('/elevator/elevator_goto_floor', ElevatorFloorGoal)
	clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
	result = execute_delivery()

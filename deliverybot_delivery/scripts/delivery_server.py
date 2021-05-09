#!/usr/bin/env python

import rospy
import sys
import os
import yaml
import tf_conversions
import actionlib
import rosparam
from std_srvs.srv import Empty
from elevator_controls.srv import ElevatorFloorGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from deliverybot_navigation.srv import MapFilePath
import deliverybot_navigation.msg


class DeliveryServer(object):

	_feedback = deliverybot_navigation.msg.DeliverFeedback()
	_result = deliverybot_navigation.msg.DeliverResult()	

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(
			self._action_name,
		deliverybot_navigation.msg.DeliverAction,
		execute_cb = self.execute_cb,
		auto_start = False
		)
		self.dbot_deliver = rospy.ServiceProxy('/dbot/deliver_package', Empty)
		self.change_map = rospy.ServiceProxy('/load_map', MapFilePath)
		self.el_open_door = rospy.ServiceProxy('/elevator/open_elevator_doors', Empty)
		self.el_close_door = rospy.ServiceProxy('/elevator/close_elevator_doors', Empty)
		self.el_change_floor = rospy.ServiceProxy('/elevator/elevator_goto_floor', ElevatorFloorGoal)
		self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
		self._as.start()
		rospy.loginfo("Delivery server started")

	def move(self, x, y, a):
		
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

	def load_map(self, map_path):
		rospy.loginfo("Waiting for service /load_map")
		self.change_map.wait_for_service()
		self.change_map.call(map_path)
		rospy.loginfo("Map Chagned.")

	def execute_action(self, action):
		# action = (type: string, value: object)
		if action[0] == "ELEVATOR_DOOR_OPEN":
			result = self.el_open_door.call()
		elif action[0] == "ELEVATOR_DOOR_CLOSE":
			result = self.el_close_door.call()
		elif action[0] == "DELIVER_PACKAGE":
			result = self.dbot_deliver.call()
		elif action[0] == "CHANGE_FLOOR":
			result = self.el_change_floor.call(action[1])
		elif action[0] == "CLEAR_COSTMAP":
			result = self.clear_costmap.call()
		elif action[0] == "WAIT":
			pass
		else:
			rospy.logwarn("Action type " + str(action[0]) + " not recognized.")

		return result
		
	def load_building_description(self, building_name):
		# TODO - replace map folder with DB
		map_folder = os.path.abspath(os.environ.get('MAPS_FOLDER_PATH'))
		with open(os.path.join(map_folder, building_name, 'building.yaml')) as f:
			yaml_desc = yaml.safe_load(f)
		return yaml_desc

	def generate_plan(self, goal):
		path = []
		rospy.loginfo("Generating Plan")
		building_desc = self.load_building_description(goal.building_name)
		# load lobby map
		path.append((
			"MAP",
			(
				os.path.abspath(os.path.join(os.environ.get('MAPS_FOLDER_PATH'), goal.building_name, 'lobby.yaml'))
			)
		))
		# elevator sequence
		path.append((
			"MOVE",
			(
				building_desc["elevator"]["out"]["x"],
				building_desc["elevator"]["out"]["y"],
				building_desc["elevator"]["out"]["a"]
			)
		))
		path.append((
			"ACTION",
			(
				"CHANGE_FLOOR",
				0
			)
		))
		path.append((
			"ACTION",
			(
				"ELEVATOR_DOOR_OPEN",
			)
		))
		path.append((
			"MOVE",
			(
				building_desc["elevator"]["in"]["x"],
				building_desc["elevator"]["in"]["y"],
				building_desc["elevator"]["in"]["a"]
			)
		))
		path.append((
			"ACTION",
			(
				"ELEVATOR_DOOR_CLOSE",
			)
		))
		path.append((
			"ACTION",
			(
				"CHANGE_FLOOR",
				goal.floor
			)
		))
		path.append((
			"ACTION",
			(
				"ELEVATOR_DOOR_OPEN",
			)
		))
		path.append((
			"MAP",
			(
				os.path.abspath(os.path.join(os.environ.get('MAPS_FOLDER_PATH'), goal.building_name, 'floor.yaml'))
			)
		))
		path.append((
			"MOVE",
			(
				building_desc["elevator"]["out"]["x"],
				building_desc["elevator"]["out"]["y"],
				-building_desc["elevator"]["out"]["a"]
			)
		))
		path.append((
			"ACTION",
			(
				"ELEVATOR_DOOR_CLOSE",
			)
		))
		path.append((
			"MOVE",
			(
				building_desc["rooms"][goal.room]["x"],
				building_desc["rooms"][goal.room]["y"],
				building_desc["rooms"][goal.room]["a"]
			)
		))
		path.append((
			"ACTION",
			(
				"DELIVER_PACKAGE",
			)
		))
		path.append((
			"MOVE",
			(
				building_desc["elevator"]["out"]["x"],
				building_desc["elevator"]["out"]["y"],
				building_desc["elevator"]["out"]["a"]
			)
		))
		path.append((
			"ACTION",
			(
				"CHANGE_FLOOR",
				goal.floor
			)
		))
		path.append((
			"ACTION",
			(
				"ELEVATOR_DOOR_OPEN",
			)
		))
		path.append((
			"ACTION",
			(
				"CLEAR_COSTMAP",
			)
		))		
		path.append((
			"MOVE",
			(
				building_desc["elevator"]["in"]["x"],
				building_desc["elevator"]["in"]["y"],
				building_desc["elevator"]["in"]["a"]
			)
		))
		path.append((
			"ACTION",
			(
				"ELEVATOR_DOOR_CLOSE",
			)
		))
		path.append((
			"ACTION",
			(
				"CHANGE_FLOOR",
				0
			)
		))
		path.append((
			"ACTION",
			(
				"ELEVATOR_DOOR_OPEN",
			)
		))
		path.append((
			"MAP",
			(
				os.path.abspath(os.path.join(os.environ.get('MAPS_FOLDER_PATH'), goal.building_name, 'lobby.yaml'))
			)
		))
		path.append((
			"MOVE",
			(
				building_desc["elevator"]["out"]["x"],
				building_desc["elevator"]["out"]["y"],
				-building_desc["elevator"]["out"]["a"]
			)
		))
		path.append((
			"ACTION",
			(
				"ELEVATOR_DOOR_CLOSE",
			)
		))
		path.append((
			"MOVE",
			(
				building_desc["lobby"]["pickup"]["x"],
				building_desc["lobby"]["pickup"]["y"],
				building_desc["lobby"]["pickup"]["a"]
			)
		))
		rospy.loginfo("Completed Generating Plan")
		return path

	def execute_cb(self, goal):
		rospy.loginfo("Goal Received.")
		success = True
		plan = self.generate_plan(goal)
		for action_type, action_value in plan:
			if action_type == "MAP":
				self.load_map(action_value)
			elif action_type == "ACTION":
				self.execute_action(action_value)
			elif action_type == "MOVE":
				self.move(*action_value)
			else:
				rospy.logerr("Unknown action type found in plan.")
		if success:
			rospy.loginfo("Successfully delivered.")
			self._as.set_succeeded(self._result)


if __name__ == "__main__":
	rospy.init_node("delivery_server")
	server = DeliveryServer(rospy.get_name())
	rospy.spin()

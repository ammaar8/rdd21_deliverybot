#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
import yaml
import sys
import tf_conversions
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(x=0, y=0, a=0):

	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
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
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	else:
		return client.get_result()

if __name__ == '__main__':
	try:
		rospy.init_node('movebase_client_py')
		goal = rospy.get_param('~goal')

		x = goal["x"]
		y = goal["y"]
		a = goal["a"]

		result = movebase_client(x, y, a)
		if result:
			rospy.loginfo("Goal execution done!")
	except rospy.ROSInterruptException:
		rospy.loginfo("Goal execution finished.")

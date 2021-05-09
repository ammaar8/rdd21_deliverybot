#!/usr/bin/env python

import rospy
import rospkg

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import os
from gazebo_msgs.srv import SpawnModel

rospy.init_node("spawn_package")
rospack = rospkg.RosPack()
spawn_package_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
models_path = os.path.join(rospack.get_path('deliverybot_simulations'), 'models', 'packages')
rospy.loginfo("Package Spawned")
pose = Pose()
pose.position.z = 0.15
package_size = "small_package"
spawn_package_client(
    model_name='package',
    model_xml=open(os.path.join(models_path, package_size + ".sdf"), 'r').read(),
    robot_namespace='',
    initial_pose = pose,
    reference_frame = "dbot::base_link"
)

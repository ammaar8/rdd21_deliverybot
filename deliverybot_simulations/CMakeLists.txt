cmake_minimum_required(VERSION 3.0.2)
project(deliverybot_simulations)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/package_spawner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
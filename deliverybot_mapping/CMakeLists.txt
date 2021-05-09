cmake_minimum_required(VERSION 3.0.2)
project(deliverybot_mapping)

find_package(catkin REQUIRED COMPONENTS
  map_server
  rospy
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/mapping_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


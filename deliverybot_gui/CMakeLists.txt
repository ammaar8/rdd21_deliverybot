cmake_minimum_required(VERSION 3.0.2)
project(deliverybot_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
)


catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/deliverybot_hw_gui.py
  scripts/launch_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


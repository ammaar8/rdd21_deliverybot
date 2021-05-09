cmake_minimum_required(VERSION 3.0.2)
project(deliverybot_control)


catkin_package(CATKIN_DEPENDS
	rospy
)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    joint_state_controller
)

include_directories(
# include
    ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
    scripts/deliverybot_hw_controller.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

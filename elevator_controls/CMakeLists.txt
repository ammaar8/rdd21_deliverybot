cmake_minimum_required(VERSION 3.0.2)
project(elevator_controls)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rospy
  controller_manager
  joint_state_controller
  robot_state_publisher
  std_msgs
  message_generation
)

add_service_files(
  FILES
  ElevatorFloorGoal.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/elevator_hw_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
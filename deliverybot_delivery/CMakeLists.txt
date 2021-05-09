cmake_minimum_required(VERSION 3.0.2)
project(deliverybot_delivery)


find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/send_goal.py
  scripts/delivery_client.py
  scripts/delivery_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


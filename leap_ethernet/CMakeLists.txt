cmake_minimum_required(VERSION 3.0.2)
project(leap_ethernet)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/leap_ethernet_node.py
  scripts/leap_ethernet_commander.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

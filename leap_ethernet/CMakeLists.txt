# Build the LEAP ROS package and install its Python nodes and launch files.
cmake_minimum_required(VERSION 3.0.2)
project(leap_ethernet)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  actionlib
  control_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs std_srvs
  actionlib
  control_msgs
)

# Keep /usr/bin/env python3 so nodes use the active Python 3 environment.
install(PROGRAMS
  scripts/leap_ethernet_node.py
  scripts/leap_ethernet_commander.py
  scripts/leap_ethernet_reboot.py
  scripts/leap_trajectory_action_server.py
  scripts/uhas_fetch_leap_controller.py
  scripts/uhas_dummy_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

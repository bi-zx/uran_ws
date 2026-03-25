cmake_minimum_required(VERSION 3.0.2)
project(uran_core)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  uran_msgs
  uran_srvs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  uran_msgs
  uran_srvs
)

catkin_install_python(PROGRAMS
  scripts/uran_core_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

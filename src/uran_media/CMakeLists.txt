cmake_minimum_required(VERSION 3.0.2)
project(uran_media)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  uran_msgs
  uran_srvs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  std_msgs
  uran_msgs
  uran_srvs
)

catkin_install_python(PROGRAMS
  scripts/uran_media_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

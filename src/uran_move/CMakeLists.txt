cmake_minimum_required(VERSION 3.0.2)
project(uran_move)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  uran_msgs
  quadrotor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  uran_msgs
  quadrotor_msgs
)

catkin_install_python(PROGRAMS
  scripts/uran_move_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

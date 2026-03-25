cmake_minimum_required(VERSION 3.0.2)
project(uran_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  StateField.msg
  StateSnapshot.msg
  HeartbeatStatus.msg
  UnifiedMoveCmd.msg
  ModeSwitchCmd.msg
  MediaSwitchCmd.msg
  UplinkProtocolCmd.msg
  UplinkPayload.msg
  MediaCtrlCmd.msg
  FrpcCtrlCmd.msg
  TaskCtrlCmd.msg
  ParamUpdateCmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

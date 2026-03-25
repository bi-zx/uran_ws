cmake_minimum_required(VERSION 3.0.2)
project(uran_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_service_files(
  FILES
  GetStateField.srv
  SetStateField.srv
  ConnectProtocol.srv
  GetNetworkStatus.srv
  SwitchMovePlugin.srv
  ReloadConfig.srv
  ListSensors.srv
  GetTaskStatus.srv
  TriggerStateReport.srv
  ConfigureStateReport.srv
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

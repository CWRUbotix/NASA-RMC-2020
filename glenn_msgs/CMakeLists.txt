cmake_minimum_required(VERSION 3.0.2)
project(glenn_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  message_generation
  message_runtime
)

add_message_files(DIRECTORY msg)
add_service_files(DIRECTORY srv)
add_action_files(DIRECTORY action)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
)

cmake_minimum_required(VERSION 2.8.3)
project(uwb_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
)

catkin_python_setup()

add_service_files(
   FILES
   EffectiveRPM.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES localization
  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

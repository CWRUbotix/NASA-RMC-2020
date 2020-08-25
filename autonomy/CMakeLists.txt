cmake_minimum_required(VERSION 2.8.3)
project(autonomy)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  message_generation
  actionlib_msgs
  actionlib
)

add_message_files(
    FILES
    TransitPath.msg
    ExtraSensorValues.msg
    TransitControlData.msg
)

add_action_files(
  FILES
  GoToGoal.action
) 

add_service_files(
  FILES
  RobotState.srv
  StartAutonomy.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autonomy
CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

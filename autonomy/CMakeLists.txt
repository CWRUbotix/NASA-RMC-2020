cmake_minimum_required(VERSION 2.8.3)
project(autonomy)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autonomy
CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

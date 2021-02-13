cmake_minimum_required(VERSION 3.0.2)
project(glenn_utilities)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

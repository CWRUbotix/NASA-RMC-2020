cmake_minimum_required(VERSION 3.0.2)
project(excavation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  glenn_msgs
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS actionlib glenn_msgs rospy
)
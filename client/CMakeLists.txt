cmake_minimum_required(VERSION 2.8.3)
project(client)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  rqt_gui
  rqt_gui_py
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS ##include
#  LIBRARIES client
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

include_directories(
 ##include
  ${catkin_INCLUDE_DIRS}
)

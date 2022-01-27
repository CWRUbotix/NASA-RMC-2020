cmake_minimum_required(VERSION 3.0.2)
project(mechs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  glenn_msgs
  rospy
  roscpp
  smacc
  ros_timer_client
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hwctrl
  CATKIN_DEPENDS actionlib glenn_msgs rospy roscpp smacc ros_timer_client
#  DEPENDS system_lib
#  CATKIN_DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sm_atomic_node src/sm_atomic_node.cpp)
add_executable(sm_deposition_node src/sm_deposition_node.cpp)

target_link_libraries(sm_atomic_node ${catkin_LIBRARIES})
target_link_libraries(sm_deposition_node ${catkin_LIBRARIES})

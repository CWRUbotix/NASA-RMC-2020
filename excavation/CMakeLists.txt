cmake_minimum_required(VERSION 3.0.2)
project(excavation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  glenn_msgs
  rospy
  roscpp
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include

  LIBRARIES excavation

  CATKIN_DEPENDS 
    actionlib 
    glenn_msgs 
    rospy 
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(behavior_tree_factory src/behavior_tree_factory.cpp)

target_link_libraries(behavior_tree_factory ${catkin_LIBRARIES})
add_dependencies(behavior_tree_factory ${catkin_EXPORTED_TARGETS})
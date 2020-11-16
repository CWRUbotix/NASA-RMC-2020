cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  nodelet
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization
  CATKIN_DEPENDS rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(point_cloud_decimator src/point_cloud_decimator.cpp)

target_link_libraries(point_cloud_decimator ${catkin_LIBRARIES})


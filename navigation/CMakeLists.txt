cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  glenn_msgs
  pcl_ros
  pcl_conversions
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    glenn_msgs
    pcl_ros
    pcl_conversions
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(point_cloud_normal_estimator src/point_cloud_normal_estimator.cpp)

target_link_libraries(point_cloud_normal_estimator ${catkin_LIBRARIES})

add_dependencies(point_cloud_normal_estimator ${catkin_EXPORTED_TARGETS})

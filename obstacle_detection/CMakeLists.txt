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
  tf2_ros
  tf2_sensor_msgs
  glenn_msgs
)

find_package(PCL REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES localization
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    pcl_conversions
    pcl_ros
    nodelet
    tf2_ros
    tf2_sensor_msgs
    glenn_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(point_cloud_decimator src/point_cloud_decimator.cpp)
add_executable(point_cloud_normal_estimator src/point_cloud_normal_estimator.cpp)
add_executable(point_cloud_icp_odometry src/point_cloud_icp_odometry.cpp)

target_link_libraries(point_cloud_decimator ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(point_cloud_normal_estimator ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(point_cloud_icp_odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(point_cloud_normal_estimator ${catkin_EXPORTED_TARGETS})
add_dependencies(point_cloud_icp_odometry ${catkin_EXPORTED_TARGETS})

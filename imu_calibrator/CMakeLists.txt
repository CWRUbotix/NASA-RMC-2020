cmake_minimum_required(VERSION 3.0.2)
project(imu_calibrator)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES imu_calibrator
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_calibrator src/imu_calibrator_node.cpp)

target_link_libraries(imu_calibrator ${catkin_LIBRARIES})

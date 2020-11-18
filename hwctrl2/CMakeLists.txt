cmake_minimum_required(VERSION 2.8.3)
project(hwctrl2)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  rospy
  message_generation
  message_runtime
)

catkin_python_setup()

add_message_files(DIRECTORY msg)

# add_service_files(DIRECTORY srv)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hwctrl
  CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hwctrl2
  src/hwctrl_node.cpp
  src/threads/sensor_thread.cpp
  src/threads/canbus_thread.cpp
  src/threads/motor_thread.cpp
  src/hardware/sensor.cpp
  src/hardware/motor.cpp
  src/hardware/quad_encoder.cpp
  src/hardware/uwb.cpp
  src/hardware/lsm6ds3.cpp
  src/hardware/ads1120.cpp
  src/hardware/adt7310.cpp
  src/hardware/vesc.cpp
  src/hardware/bmc.cpp
  src/interface/gpio.cpp
  src/interface/spi.cpp
  src/util.cpp
)

add_executable(hwctrl2_cal
  src/hwctrl_cal_node.cpp
  src/threads/sensor_thread.cpp
  src/threads/sensor_cal_thread.cpp
  src/threads/canbus_thread.cpp
  src/threads/motor_thread.cpp
  src/hardware/sensor.cpp
  src/hardware/motor.cpp
  src/hardware/quad_encoder.cpp
  src/hardware/uwb.cpp
  src/hardware/lsm6ds3.cpp
  src/hardware/ads1120.cpp
  src/hardware/adt7310.cpp
  src/hardware/vesc.cpp
  src/hardware/bmc.cpp
  src/interface/gpio.cpp
  src/interface/spi.cpp
  src/util.cpp
)

target_link_libraries(hwctrl2 ${catkin_LIBRARIES})
target_link_libraries(hwctrl2_cal ${catkin_LIBRARIES})

# for hwctrl executable
add_dependencies(hwctrl2 ${catkin_EXPORTED_TARGETS})
add_dependencies(hwctrl2 hwctrl2_generate_messages_cpp)

# for hwctrl_cal executable
add_dependencies(hwctrl2_cal ${catkin_EXPORTED_TARGETS})
add_dependencies(hwctrl2_cal hwctrl2_generate_messages_cpp)

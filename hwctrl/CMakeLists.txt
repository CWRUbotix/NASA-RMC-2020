cmake_minimum_required(VERSION 3.7.2)
project(hwctrl)

add_compile_options(-std=c++14 -O2)

find_package(Boost 1.62 REQUIRED COMPONENTS system filesystem)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  message_runtime
)

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

add_executable(hwctrl
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

add_executable(hwctrl_cal
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

set_property(TARGET hwctrl PROPERTY INTERPROCEDURAL_OPTIMIZATION True)
set_property(TARGET hwctrl_cal PROPERTY INTERPROCEDURAL_OPTIMIZATION True)

target_link_libraries(hwctrl ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(hwctrl_cal ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# for hwctrl executable
add_dependencies(hwctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(hwctrl hwctrl_generate_messages_cpp)

# for hwctrl_cal executable
add_dependencies(hwctrl_cal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(hwctrl_cal hwctrl_generate_messages_cpp)

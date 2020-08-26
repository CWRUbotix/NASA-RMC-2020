cmake_minimum_required(VERSION 2.8.3)
project(hwctrl)

add_compile_options(-std=c++11)

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

add_service_files(DIRECTORY srv)

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
  src/buffer.cpp
  src/canbus.cpp
  src/crc.cpp
  src/gpio.cpp
  src/hwctrl_node.cpp
  src/hwctrl.cpp
  src/lsm6ds3.cpp
  src/motors.cpp
  src/parse_csv.cpp
  src/sensors.cpp
  src/spi.cpp
  src/uwb.cpp
  src/vesc.cpp
)

add_executable(hwctrl_cal
  src/buffer.cpp
  src/canbus.cpp
  src/crc.cpp
  src/gpio.cpp
  src/hwctrl_cal.cpp
  src/hwctrl.cpp
  src/lsm6ds3.cpp
  src/motors.cpp
  src/parse_csv.cpp
  src/sensors.cpp
  src/spi.cpp
  src/uwb.cpp
  src/vesc.cpp
)

target_link_libraries(hwctrl ${catkin_LIBRARIES})
target_link_libraries(hwctrl_cal ${catkin_LIBRARIES})

# for hwctrl executable
add_dependencies(hwctrl ${catkin_EXPORTED_TARGETS})
add_dependencies(hwctrl hwctrl_generate_messages_cpp)

# for hwctrl_cal executable
add_dependencies(hwctrl_cal ${catkin_EXPORTED_TARGETS})
add_dependencies(hwctrl_cal hwctrl_generate_messages_cpp)

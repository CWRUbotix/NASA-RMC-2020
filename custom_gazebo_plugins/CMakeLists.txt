cmake_minimum_required(VERSION 3.0.2)
project(custom_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS tf)
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

catkin_package()

add_library(custom_p3d_plugin SHARED custom_p3d_plugin.cpp)
target_link_libraries(custom_p3d_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS custom_p3d_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(gazebo REQUIRED)
find_package(tf REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${tf_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${tf_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(custom_p3d_plugin SHARED custom_p3d_plugin.cpp)
target_link_libraries(custom_p3d_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${tf_LIBRARIES})
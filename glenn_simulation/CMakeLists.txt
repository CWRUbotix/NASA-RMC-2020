cmake_minimum_required(VERSION 3.0.2)
project(glenn_simulation)

find_package(
catkin REQUIRED COMPONENTS
  rospy
  tf

gazebo REQUIRED
)

catkin_package(
  CATKIN_DEPENDS 
    rospy
)

catkin_python_setup()


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(custom_p3d_plugin SHARED src/glenn_simulation/custom_gazebo_plugins/custom_p3d_plugin.cpp)
target_link_libraries(custom_p3d_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS custom_p3d_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

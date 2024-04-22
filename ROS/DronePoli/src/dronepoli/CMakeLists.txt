cmake_minimum_required(VERSION 3.0.2)
project(dronepoli)

find_package(catkin REQUIRED COMPONENTS
  mavros
  mavros_msgs
  roscpp
  rospy
  std_msgs
  darknet_ros
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(dronepoli
  src/controller.cpp
)

add_dependencies(dronepoli ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable (dronepoli_node
  include/drone.h
  include/_keyboard.h
  include/_convert.h
  include/callback.h
  src/controller.cpp
)
target_link_libraries(dronepoli_node dronepoli ${catkin_LIBRARIES})
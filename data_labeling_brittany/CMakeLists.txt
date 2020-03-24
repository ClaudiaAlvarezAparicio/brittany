cmake_minimum_required(VERSION 2.8.3)
project(data_labeling_brittany)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_filters
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES data_labeling_brittany
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_filters
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_executable(data_labeling_brittany src/data_labeling_brittany.cpp)
add_dependencies(data_labeling_brittany ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_labeling_brittany ${catkin_LIBRARIES})

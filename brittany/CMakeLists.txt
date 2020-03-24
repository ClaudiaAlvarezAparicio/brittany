cmake_minimum_required(VERSION 2.8.3)
project(brittany)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  roslib
  tensorflow_ros_cpp
  OpenCV
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${tensorflow_ros_cpp_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_executable(brittany src/brittany.cpp
                                        src/networkPrediction.cpp)
add_dependencies(brittany ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(brittany ${tensorflow_ros_cpp_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})

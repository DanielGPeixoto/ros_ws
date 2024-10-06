cmake_minimum_required(VERSION 2.8.3)
project(message_generator)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

## Declare ROS messages and services
add_message_files(
  FILES
  sensor_messages.msg  # Ensure this file exists in the 'msg' folder
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime  std_msgs sensor_msgs
)

## Build talker
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/message_generator.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker ${PROJECT_NAME}_generate_messages_cpp)  


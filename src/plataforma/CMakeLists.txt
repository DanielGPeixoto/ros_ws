cmake_minimum_required(VERSION 3.0.2)
project(plataforma)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_control_node src/motor_control_node.cpp)

target_link_libraries(motor_control_node ${catkin_LIBRARIES} lwiringPi)

add_executable(tele_op_keyboard_node src/tele_op_keyboard_node.cpp)
target_link_libraries(tele_op_keyboard_node ${catkin_LIBRARIES})

install(TARGETS motor_control_node tele_op_keyboard_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(conversion_services)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_service_files(
  FILES
  d2r.srv
  ecef2g.srv
  g2ecef.srv
  r2d.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Add any message dependencies here
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(d2r_server src/d2r_server.cpp)
add_dependencies(d2r_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(d2r_server ${catkin_LIBRARIES})

add_executable(ecef2g_server src/ecef2g_server.cpp)
add_dependencies(ecef2g_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ecef2g_server ${catkin_LIBRARIES})

add_executable(g2ecef_server src/g2ecef_server.cpp)
add_dependencies(g2ecef_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(g2ecef_server ${catkin_LIBRARIES})

add_executable(r2d_server src/r2d_server.cpp)
add_dependencies(r2d_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(r2d_server ${catkin_LIBRARIES})


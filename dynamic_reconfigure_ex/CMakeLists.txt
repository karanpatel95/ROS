cmake_minimum_required(VERSION 2.8.3)
project(dynamic_reconfigure_ex)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/example.cfg
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(server_node src/server.cpp)
add_dependencies (server_node ${PROJECT_NAME}_gencfg)

target_link_libraries(server_node
  ${catkin_LIBRARIES}
)
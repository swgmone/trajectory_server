cmake_minimum_required(VERSION 2.8.3)
project(trajectory_server_v2)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
  message_generation
  poly_lib
)

find_package(Eigen3 REQUIRED)
find_package(nlopt REQUIRED)

add_message_files(
  DIRECTORY
    msg
  FILES
    Constraint.msg
    Waypoint.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    PlanPolyTraj.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_server_v2
  CATKIN_DEPENDS roscpp rospy std_msgs poly_lib message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/trajectory_server.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(trajectory_server_v2
  ${catkin_LIBRARIES}
#  /usr/local/lib/libnlopt.so
  ${nlopt_LIBRARIES}
)

## Declare a C++ executable
add_executable(trajectory_server_node src/trajectory_server_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(trajectory_server_node
  ${catkin_LIBRARIES}
  trajectory_server_v2
  ${nlopt_LIBRARIES}
)

add_dependencies(trajectory_server_node ${PROJECT_NAME}_generate_messages_cpp)


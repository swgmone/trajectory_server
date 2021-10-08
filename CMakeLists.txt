cmake_minimum_required(VERSION 3.0.2)
project(trajectory_server)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)

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
  LIBRARIES trajectory_server
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
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
  src/poly_lib.cpp
  src/rpoly/rpoly_ak1.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(trajectory_server
  ${catkin_LIBRARIES}
  ${NLOPT_LIBRARIES}
)

## Declare a C++ executable
add_executable(trajectory_server_node src/trajectory_server_node.cpp)
add_executable(trajectory_test_node src/trajectory_test_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(trajectory_server_node
  ${catkin_LIBRARIES}
  ${NLOPT_LIBRARIES}
  trajectory_server
)

target_link_libraries(trajectory_test_node
  ${catkin_LIBRARIES}
  ${NLOPT_LIBRARIES}
  trajectory_server
)

add_dependencies(trajectory_server_node ${PROJECT_NAME}_generate_messages_cpp)


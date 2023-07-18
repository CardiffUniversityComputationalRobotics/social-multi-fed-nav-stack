cmake_minimum_required(VERSION 3.0.2)
project(smf_move_base_control)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  laser_geometry
  tf
  roslib
  smf_move_base_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs smf_move_base_msgs
  DEPENDS Eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)
add_definitions(${EIGEN_DEFINITIONS})

add_executable(guidance_node
  include/guidance/guidance.h
  src/guidance.cpp
  src/guidance_node.cpp
)
add_dependencies(guidance_node smf_move_base_msgs_generate_messages_cpp ${smf_move_base_msgs_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(guidance_node
  ${catkin_LIBRARIES}
)
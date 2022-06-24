cmake_minimum_required(VERSION 3.0.2)
project(smf_move_base_mapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  smf_move_base_planning
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES smf_move_base_mapping
 CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(smf_move_base_mapper
src/social_costmap.cpp
# src/world_modeler.cpp
)
target_link_libraries(smf_move_base_mapper ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(smf_move_base_mapper ${catkin_EXPORTED_TARGETS} smf_move_base_msgs_generate_messages_cpp ${smf_move_base_msgs_EXPORTED_TARGETS})


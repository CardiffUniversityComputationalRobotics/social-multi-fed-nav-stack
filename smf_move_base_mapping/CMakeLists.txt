cmake_minimum_required(VERSION 3.0.2)
project(smf_move_base_mapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  smf_move_base_msgs
  pedsim_msgs
  tf
  tf2
  tf2_ros
  tf2_eigen
  grid_map_core
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_ros
  grid_map_rviz_plugin
  grid_map_sdf
  grid_map_visualization
)

find_package(Octomap REQUIRED)
find_package(ompl REQUIRED)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES smf_move_base_mapping
 CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs tf tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}

)

add_executable(smf_move_base_mapper
src/social_heatmap.cpp
src/world_modeler.cpp
)
target_link_libraries(smf_move_base_mapper ${catkin_LIBRARIES} ${Boost_LIBRARIES} ccd ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})
# add_dependencies(smf_move_base_mapper ${catkin_EXPORTED_TARGETS} smf_move_base_msgs_generate_messages_cpp ${smf_move_base_msgs_EXPORTED_TARGETS})


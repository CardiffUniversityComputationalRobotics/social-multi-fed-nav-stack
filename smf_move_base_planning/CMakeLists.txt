cmake_minimum_required(VERSION 3.0.2)
project(smf_move_base_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
  tf2_eigen
  actionlib
  actionlib_msgs
  smf_move_base_msgs
  pedsim_msgs
  roslib
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
#  LIBRARIES
  CATKIN_DEPENDS roscpp roslib rospy sensor_msgs std_msgs tf tf2 tf2_ros actionlib smf_move_base_msgs 
  # pedsim_msgs
  DEPENDS OMPL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)
link_directories(${OCTOMAP_LIBRARY_DIRS} ${ompl_LIBRARY_DIRS} ${CCD_LIBRARY_DIRS})


add_executable(smf_move_base_planner
src/planner/RRTstarMod.cpp
src/planner/InformedRRTstarMod.cpp
src/sampler/PathLengthDirectInfSamplerMod.cpp
src/planning_framework_main.cpp
src/state_validity_checker_grid_map_R2.cpp
src/local_state_validity_checker_grid_map_R2.cpp
src/new_state_sampler.cpp
src/informed_new_state_sampler.cpp
src/state_cost_objective.cpp
)
target_link_libraries(smf_move_base_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES} ccd ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})
add_dependencies(smf_move_base_planner ${catkin_EXPORTED_TARGETS} smf_move_base_msgs_generate_messages_cpp ${smf_move_base_msgs_EXPORTED_TARGETS})

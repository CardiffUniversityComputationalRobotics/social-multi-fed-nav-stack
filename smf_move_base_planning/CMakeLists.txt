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
)

find_package(Octomap REQUIRED)
find_package(ompl REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL_PC REQUIRED fcl)
# find *absolute* paths to LIBFCL_* paths
find_path(LIBFCL_INCLUDE_DIRS fcl/config.h HINTS ${LIBFCL_PC_INCLUDE_DIR} ${LIBFCL_PC_INCLUDE_DIRS})
set(LIBFCL_LIBRARIES)
foreach(_lib ${LIBFCL_PC_LIBRARIES})
  find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
endforeach()

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
  ${FCL_INCLUDE_DIRS}
)
link_directories(${OCTOMAP_LIBRARY_DIRS} ${ompl_LIBRARY_DIRS} ${FCL_LIBRARY_DIRS} ${CCD_LIBRARY_DIRS})


add_executable(smf_move_base_planner
src/planner/RRTstarMod.cpp
src/planning_framework_main.cpp
src/state_validity_checker_octomap_fcl_R2.cpp
src/new_state_sampler.cpp
src/state_cost_objective.cpp
)
target_link_libraries(smf_move_base_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES} ccd ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})
add_dependencies(smf_move_base_planner ${catkin_EXPORTED_TARGETS} smf_move_base_msgs_generate_messages_cpp ${smf_move_base_msgs_EXPORTED_TARGETS})

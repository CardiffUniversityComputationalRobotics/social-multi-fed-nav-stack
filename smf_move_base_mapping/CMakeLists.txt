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
#  LIBRARIES smf_move_base_mapping
 CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs tf tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)

add_executable(smf_move_base_mapper
src/social_costmap.cpp
src/world_modeler.cpp
)
target_link_libraries(smf_move_base_mapper ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES} ccd ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})
# add_dependencies(smf_move_base_mapper ${catkin_EXPORTED_TARGETS} smf_move_base_msgs_generate_messages_cpp ${smf_move_base_msgs_EXPORTED_TARGETS})


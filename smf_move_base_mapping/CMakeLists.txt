cmake_minimum_required(VERSION 3.8)
project(smf_move_base_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(smf_move_base_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(grid_map_loader REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_octomap REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_rviz_plugin REQUIRED)
find_package(grid_map_sdf REQUIRED)
find_package(grid_map_visualization REQUIRED)
find_package(ompl REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(smf_move_base_mapper
src/social_heatmap.cpp
src/world_modeler.cpp
)
target_link_libraries(smf_move_base_mapper ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})

ament_package()

# Feedback Social Comfort Robot Navigation Framework

This is an online social robot navigation framework for indoor social scenarios that uses a Social Heatmap to represent crowded areas in an environment and also uses a multilayer technique for the planning module.

It is composed of four different packages:

- `smf_move_base_control`: it is a simple differential control method.
- `smf_move_base_mapping`: is in charge of the perception of the space.
- `smf_move_base_planning`: responsible for finding solution paths for the navigation query requested.
- `smf_move_base_msgs`: contains the messages needed for the framework and the start-goal queries.

Developed and tested on **ROS 2 Jazzy**. The `humble-devel` branch is the one that builds and runs on Jazzy; there is no `jazzy` branch.

## Requirements

- ROS 2 Jazzy on Ubuntu 24.04.
- `pedsim_msgs`, from [`pedsim_ros`](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros). It has no rosdep rule, so it has to be in the same workspace before `rosdep install` runs.
- Everything else is resolved by rosdep: OMPL, `grid_map` (`grid_map_core`, `grid_map_cv`, `grid_map_filters`, `grid_map_msgs`, `grid_map_octomap`, `grid_map_ros`, `grid_map_sdf`, `grid_map_visualization`), OctoMap and `octomap_server`, `pcl_ros`, `laser_geometry`, `tf2_ros`.

## Installation

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

git clone -b humble-devel https://github.com/CardiffUniversityComputationalRobotics/social-multi-fed-nav-stack.git
git clone -b jazzy https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Clone `pedsim_ros` before running `rosdep install`, otherwise it cannot resolve `pedsim_msgs`.

## Usage

The three nodes take their whole configuration from ROS 2 parameter files and have no defaults worth running with, so they are always started from a launch file that supplies one:

| Package | Executable | Node name used downstream |
| --- | --- | --- |
| `smf_move_base_mapping` | `smf_move_base_mapper` | `smf_move_base_mapper` |
| `smf_move_base_planning` | `smf_move_base_planner` | `smf_move_base_planner` |
| `smf_move_base_control` | `base_controller` | `smf_move_base_controller` |

This repository ships no runnable launch file. The `*.launch` files under each package are ROS 1 XML, are not installed, and `ros2 launch` cannot parse them. A complete, working launch file and parameter set for all three nodes lives in [`social_nav_benchmark`](https://github.com/CardiffUniversityComputationalRobotics/social_nav_benchmark), which is also how the stack is benchmarked:

```bash
ros2 launch social_nav_benchmark benchmark.launch.py nav_stack:=smf version:=small_house robot:=stretch
```

To run a node on its own, pass a parameter file explicitly:

```bash
ros2 run smf_move_base_mapping smf_move_base_mapper --ros-args \
  --params-file /absolute/path/to/smf_move_base_mapping.yaml
```

The parameter files in `social_nav_benchmark/config/nav_stacks/smf_nav_stack/` are a working starting point.

The planner needs a 3D map before it can plan, so start the mapper first, then the controller, then the planner. The planner also waits for the controller to report itself on `control_active_topic` before it starts following a solution.

![FrameworkConnections](https://i.imgur.com/JgMGqJW.png)

- [Feedback Social Comfort Robot Navigation Framework](#feedback-social-comfort-robot-navigation-framework)
  - [World Modeling (`smf_move_base_mapping`)](#world-modeling-smf_move_base_mapping)
    - [Parameters](#parameters)
    - [Subscribers](#subscribers)
    - [Publishers](#publishers)
    - [Services](#services)
  - [Online Social Robot Path Planning (`smf_move_base_planning`)](#online-social-robot-path-planning-smf_move_base_planning)
    - [Parameters](#parameters-1)
    - [Subscribers](#subscribers-1)
    - [Publishers](#publishers-1)
    - [Actions](#actions)
  - [Path Following Control (`smf_move_base_control`)](#path-following-control-smf_move_base_control)
    - [Parameters](#parameters-2)
    - [Subscribers](#subscribers-2)
    - [Publishers](#publishers-2)

## World Modeling (`smf_move_base_mapping`)

This package is in charge of the perception for the robot. For that, a depth camera is used, which helps to generate a 3D map of the space by using Octomap.

This 3D map is combined in a multilayer GridMap with the position of the social agents detected in the space.

Alditionally, this package also provides an idea of the crowded areas in the social environment.

### Parameters

- resolution (double, default: 1.0)

  Octomap and Grid Map resolution.

- map_frame (string, default: "map")

- fixed_frame (string, default: "fixed_frame")

  Fixed frame considered for the mapping, can be `odom` for example.

- robot_frame (string, default: "/robot_frame")

  Base frame considered from the robot for the mapping.

- offline_octomap_path (string, default: "")

  If set, that map is loaded and no live mapping is done: the laser and point cloud subscribers are never created.

- visualize_free_space (bool, default: False)

  Wether you would like to see the 3D map and grid_map. If `False`, nothing is published at `/smf_move_base_mapping/social_grid_map` and `/smf_move_base_mapping/octomap_map`.

- mapping_max_range (double, default: 5.0)

  Maximum distance to generate the 3D map from the obtained depth points.

- odometry_topic (string, default: "/odometry_topic")

- rviz_timer (double, default: 0.0)

  The time delay to publish messages to RViz. Not used if `visualize_free_space` is `False`.

- point_cloud_topic (string, default: "")

  Pointcloud topic used to generate the 3D map. Singular in ROS 2, unlike the ROS 1 version which took a list.

- point_cloud_frame (string, default: "")

  Frame of that pointcloud. It must exist in TF, otherwise nothing is mapped.

- laser_scan_topic (string, default: "")

  Laser scan topic used to generate the 3D map.

- laser_scan_frame (string, default: "")

  Frame of that laser scan.

- minimum_range (double, default: -1.0)

  Range below which scan returns are discarded. `-1.0` disables the filter.

- min_z_pc (double, default: 0.05) / max_z_pc (double, default: 1.0)

  Height band of the pointcloud that is inserted into the map.

- social_comfort_amplitude (double, default: 6.0)

  Amplitude of the social comfort layer of the grid map.

- erase_map_topic (string, default: "erase_map")

  A `True` message on this topic clears the map and skips sensor callbacks for 3 seconds. This is what resets the map between benchmark runs.

- social_agents_topic (string, default: "/pedsim_simulator/simulated_agents")

  Topic with social agents states.

- social_agent_radius (double, default: 0.4)

  Radius considered for the social agents.

- social_relevance_validity_checking (bool, default: False)

  Whether or not it is desired to only consider the relevant agents.

- robot_distance_view_max (double, default: 6.0)

  Maximum distance for the robot to consider social agents.

- robot_distance_view_min (double, default: 1.5)

  Mininum distance for the robot to consider social agents.

- robot_angle_view (double, default: 1.57)

  Maximum angle in which the social agents are considered from the field of view of the robot. Defined in radians.

- robot_velocity_threshold (double, default: 0.3)

  Maximum velocity that the robot can have.

- social_heatmap_decay_factor (double, default: 65.0)

  It is recommended a value in between 60 and 75.

### Subscribers

The name of the subscribers' topics are just defined as an example, but they may be configured using the parameters defined before.

- /pedsim_simulator/simulated_agents ([pedsim_msgs/AgentStates](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/jazzy/pedsim_msgs/msg/AgentStates.msg))

  Position, orientation, velocity and other states of the social agents.

- /pepper/camera/depth/points ([sensor_msgs/PointCloud2](https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/PointCloud2.html))

  Depth points from depth camera.

- /pepper/odom_groundtruth ([nav_msgs/Odometry](https://docs.ros.org/en/jazzy/p/nav_msgs/interfaces/msg/Odometry.html))

  Robot odometry (in this case it was Pepper robot).

### Publishers

The name of the publishers' topics are just defined as an example, but they may be configured using the parameters defined before.

- /smf_move_base_mapping/octomap_map ([visualization_msgs/MarkerArray](https://docs.ros.org/en/jazzy/p/visualization_msgs/interfaces/msg/MarkerArray.html))

  Visual representation of the 3D map created.

- /smf_move_base_mapping/social_grid_map ([grid_map_msgs/GridMap](https://docs.ros.org/en/jazzy/p/grid_map_msgs/interfaces/msg/GridMap.html))

  Grid map with 4 layers, one for obstacles and one including obstacles and social agents, one for comfort and another for social heatmap.

### Services

- /smf_move_base_mapping/get_grid_map ([grid_map_msgs/GetGridMap](https://docs.ros.org/en/jazzy/p/grid_map_msgs/interfaces/srv/GetGridMap.html))

  Fetches the grid map from the World Modeling.

## Online Social Robot Path Planning (`smf_move_base_planning`)

This package is in charge of finding socially acceptable solution paths for the robot to follow.

### Parameters

All of the following parameters have to be defined since they have no default values.

- world_frame (string)

  Main parent frame to be used as reference.

- planner_name (string)

  Planner to be used for global, can be RRT, RRTstar or PRMstar.

- local_planner_name (string)

  Planner to be used for local, can be RRT, RRTstar or PRMstar, InformedRRTstarMod, InformedRRTstar.

- planning_bounds_x (list: [x_min, x_max])

  Limits for planning in the X axis.

- planning_bounds_y (list: [y_min, y_max])

  Limits for planning in the Y axis.

- dynamic_bounds (bool)

  Whether it is wanted to change the planning bounds dinamically depending on the navigation query.

- start_state (list, [X, Y, Yaw])

  Define start state for navigation query if the action is not going to be used.

- goal_state (list, [X, Y, Yaw])

  Define goal state for navigation query if no action is used.

- timer_period (double)

  Controls the time for sleeps in the node.

- solving_time (double)

  Time given to the planner to solve the navigation query, both for the global and the local layer.

- opport_collision_check (bool)

  Whether it is wanted to use opportunistic collision checking or not.

- reuse_last_best_solution (bool)

  Whether it is wanted to apply the reuse of the last best known solution for the global planner.

- local_reuse_last_best_solution (bool)

  Whether it is wanted to apply the reuse of the last best known solution for the local planner.

- optimization_objective (string)

  Defines which optimization objective to use is desired for the global planner. Currently, only supported PathLength and SocialHeatmap.

- local_optimization_objective (string)

  Defines which optimization objective to use is desired for the local planner. Currently, only supported PathLength and SocialComfort.

- motion_cost_interpolation (bool)

  If motion cost interpolation is desired.

- xy_goal_tolerance (double)

  Tolerance in the XY axis for the global goal state.

- local_xy_goal_tolerance (double)

  Tolerance in the XY axis for the local goal state.

- visualize_tree: (bool)

  Whether it is desired to visualize or not the tree generated by the planner.

- grid_map_service (string)

  Service name to get the GridMap.

- control_active_topic (string)

  Topic to communicate with the control module to know when control is ready.

- robot_base_radius (double)

  Radius of the base of the robot to be considered for the planning.

- odometry_topic (string)

- query_goal_topic (string)

  Topic to request a query without using actions.

- goto_action (string)

  Action name to run a navigation query.

- local_path_range (double)

  Distance from the robot position to the decided local goal.

- global_time_percent (double)

  Percentage given to the global planner from the time budget.

### Subscribers

The name of the subscribers' topics are just defined as an example, but they may be configured using the parameters defined before.

- /smf_move_base_planner/query_goal ([geometry_msgs/PoseStamped](https://docs.ros.org/en/jazzy/p/geometry_msgs/interfaces/msg/PoseStamped.html))
- /pepper/odom_groundtruth ([nav_msgs/Odometry](https://docs.ros.org/en/jazzy/p/nav_msgs/interfaces/msg/Odometry.html))

  Robot odometry.

- /control_active_topic ([std_msgs/Bool](https://docs.ros.org/en/jazzy/p/std_msgs/interfaces/msg/Bool.html))

### Publishers

The name of the publishers' topics are just defined as an example, but they may be configured using the parameters defined before.

- /smf_move_base_planner/smf_move_base_solution_path ([smf_move_base_msgs/Path2D](https://github.com/CardiffUniversityComputationalRobotics/social-multi-fed-nav-stack/blob/humble-devel/smf_move_base_msgs/msg/Path2D.msg))

  Solution path found by the planner. It is passed to the control module.

- /smf_move_base_planner/smf_num_nodes ([std_msgs/Int32](https://docs.ros.org/en/jazzy/p/std_msgs/interfaces/msg/Int32.html))

  Number of valid nodes sampled by the local planner.

- /smf_move_base_planner/query_goal_pose_rviz ([geometry_msgs/PoseStamped](https://docs.ros.org/en/jazzy/p/geometry_msgs/interfaces/msg/PoseStamped.html))

  Shows in RViz the query pose requested to the planner.

- /smf_move_base_planner/query_goal_radius_rviz ([visualization_msgs/Marker](https://docs.ros.org/en/jazzy/p/visualization_msgs/interfaces/msg/Marker.html))

  Shows in RViz the radius of the query requested to the planner.

- /smf_move_base_planner/solution_path ([visualization_msgs/Marker](https://docs.ros.org/en/jazzy/p/visualization_msgs/interfaces/msg/Marker.html))

  Visual solution path to be seen in RViz.

### Actions

- `/goto_action` ([smf_move_base_msgs/GoTo2D](https://github.com/CardiffUniversityComputationalRobotics/social-multi-fed-nav-stack/blob/humble-devel/smf_move_base_msgs/action/Goto2D.action))

  Action to request navigation query. The name of the action server depends on the parameter `goto_action`.

## Path Following Control (`smf_move_base_control`)

This module is in charge of sending the velocities to the robot according to the path obtained so that the robot goes through that path.

### Parameters

- max_vel (double, default: 0.1)

  Maximum velocity the robot can have in the X axis.

- min_vel (double, default: 0.05)

  Minimum velocity for the robot to move in the X axis.

- max_turn_rate (double, default: 0.5)

  Maximum velocity with which the robot can rotate.

- min_turn_rate (double, default: 0.5)

  Minimum velocity for the robot to rotate.

- controller_hz (double, default: 100)

  Frequency in which velocities are sent

- control_path_topic (string, default: "/control_path_topic")

  Topic in which the desired path to follow is published.

- control_output_topic (string, default: "/control_output_topic")

  Topic in which the velocities are published.

- odometry_topic (string, default: "/odometry_topic")

### Subscribers

The name of the subscribers' topics are just defined as an example, but they may be configured using the parameters defined before.

- /smf_move_base_planner/smf_move_base_solution_path ([smf_move_base_msgs/Path2D](https://github.com/CardiffUniversityComputationalRobotics/social-multi-fed-nav-stack/blob/humble-devel/smf_move_base_msgs/msg/Path2D.msg))
- /pepper/odom_groundtruth ([nav_msgs/Odometry](https://docs.ros.org/en/jazzy/p/nav_msgs/interfaces/msg/Odometry.html))

### Publishers

The name of the publishers' topics are just defined as an example, but they may be configured using the parameters defined before.

- /control_active_topic ([std_msgs/Bool](https://docs.ros.org/en/jazzy/p/std_msgs/interfaces/msg/Bool.html))
- /pepper/cmd_vel ([geometry_msgs/Twist](https://docs.ros.org/en/jazzy/p/geometry_msgs/interfaces/msg/Twist.html))



#ifndef OMPL_CONTRIB_LOCAL_STATE_VALIDITY_CHECKER_GRID_MAP_R2_
#define OMPL_CONTRIB_LOCAL_STATE_VALIDITY_CHECKER_GRID_MAP_R2_

// ROS
#include <ros/ros.h>
// ROS markers rviz
#include <visualization_msgs/Marker.h>
// ROS messages
#include <nav_msgs/Odometry.h>
// ROS tf
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

// Standard libraries
#include <cstdlib>
#include <cmath>
#include <string>

// grid map library
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/GridMap.h>

// OMPL
#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/debug/Profiler.h>

// Boost
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

// Eigen
#include <Eigen/Dense>

#include <iostream>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// #include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <math.h>

// ROS-GridMap interface
using grid_map_msgs::GetGridMap;
// Standard namespace
using namespace std;
// OMPL namespaces
namespace ob = ompl::base;
namespace og = ompl::geometric;

//!  LocalGridMapStateValidityCheckerR2 class.
/*!
  Octomap State Validity checker.
  Extension of an abstract class used to implement the state validity checker over an octomap using FCL.
*/
class LocalGridMapStateValidityCheckerR2 : public ob::StateValidityChecker
{
public:
  //! LocalGridMapStateValidityCheckerR2 constructor.
  /*!
   * Besides of initializing the private attributes, it loads the octomap.
   */
  LocalGridMapStateValidityCheckerR2(const ob::SpaceInformationPtr &si, const bool opport_collision_check,
                                     std::vector<double> planning_bounds_x, std::vector<double> planning_bounds_y);

  //! LocalGridMapStateValidityCheckerR2 destructor.
  /*!
   * Destroy the octomap.
   */
  ~LocalGridMapStateValidityCheckerR2();

  //! State validator.
  /*!
   * Function that verifies if the given state is valid (i.e. is free of collision) using FCL
   */
  virtual bool isValid(const ob::State *state) const;

  /*
   * Returns the cost value for the integration of the path defined on the equation that defines an extended
   * social comfort zone model.
   */
  virtual double checkExtendedSocialComfort(const ob::State *state,
                                            const ob::SpaceInformationPtr space) const;

  virtual bool isValidPoint(const ob::State *state) const;

private:
  // ROS
  ros::NodeHandle nh_, local_nh_;

  double grid_map_min_x_, grid_map_min_y_, grid_map_min_z_;
  double grid_map_max_x_, grid_map_max_y_, grid_map_max_z_;
  std::vector<double> planning_bounds_x_, planning_bounds_y_;
  double robot_base_radius_;
  std::string grid_map_service_, state_space_;
  grid_map_msgs::GridMap grid_map_msgs_;
  grid_map::GridMap grid_map_;

  bool opport_collision_check_, local_use_social_heatmap_;

  grid_map::Matrix obstacles_grid_map_;
  grid_map::Matrix full_grid_map_;
  grid_map::Matrix comfort_grid_map_;
  grid_map::Matrix social_heatmap_grid_map_;
  grid_map::Matrix risk_zones_grid_map_;
};

#endif

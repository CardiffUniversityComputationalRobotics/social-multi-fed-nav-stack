

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

// Octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

// OMPL
#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/debug/Profiler.h>

// Boost
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

// Eigen
#include <Eigen/Dense>

// Social Costmap
#include <nav_msgs/OccupancyGrid.h>

#include <iostream>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// #include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <math.h>

// ROS-Octomap interface
using octomap_msgs::GetOctomap;
// Standard namespace
using namespace std;
// Octomap namespace
using namespace octomap;
// OMPL namespaces
namespace ob = ompl::base;
namespace og = ompl::geometric;

//!  LocalOmFclStateValidityCheckerR2 class.
/*!
  Octomap State Validity checker.
  Extension of an abstract class used to implement the state validity checker over an octomap using FCL.
*/
class LocalOmFclStateValidityCheckerR2 : public ob::StateValidityChecker
{
public:
  //! LocalOmFclStateValidityCheckerR2 constructor.
  /*!
   * Besides of initializing the private attributes, it loads the octomap.
   */
  LocalOmFclStateValidityCheckerR2(const ob::SpaceInformationPtr &si, const bool opport_collision_check,
                                   std::vector<double> planning_bounds_x, std::vector<double> planning_bounds_y);

  //! LocalOmFclStateValidityCheckerR2 destructor.
  /*!
   * Destroy the octomap.
   */
  ~LocalOmFclStateValidityCheckerR2();

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

  // Octomap
  octomap::AbstractOcTree *abs_octree_;
  octomap::OcTree *octree_;
  double octree_min_x_, octree_min_y_, octree_min_z_;
  double octree_max_x_, octree_max_y_, octree_max_z_;
  std::vector<double> planning_bounds_x_, planning_bounds_y_;
  double robot_base_radius_, robot_base_height_;
  std::string octomap_service_;

  // cost objective type
  std::string optimization_objective;

  // topics
  std::string sim_agents_topic;
  std::string odometry_topic;
  std::string social_costmap_topic_;

  // extra frames
  std::string main_frame;

  double octree_res_;

  bool opport_collision_check_, social_relevance_validity_checking_, use_social_costmap_;

  // pedsim variables
  pedsim_msgs::AgentStatesConstPtr agentStates;

  // relevant agents
  pedsim_msgs::AgentStates relevant_agent_states_;

  // odometry data
  nav_msgs::OdometryConstPtr odomData;

  //! basic social personal space parameters defined
  /*
   * amplitude of basic social personal space function
   */
  double Ap = 100;

  /*
   * standard deviation in X of gaussian basic social personal space function
   */
  double sigmaX = 0.45;

  /*
   * standard deviation in X of gaussian basic social personal space function
   */
  double sigmaY = 0.45;

  // public:
  /*
   * distance between robot and agent
   */
  // double dRobotAgent = 1;
  // double tethaAgent;

  //! extra parameters for social space model

  /*
   * normalization factor, multiplied by agent velocity
   */
  double fv = 0.8;

  /*
   * frontal area factor, sums with rest of factors
   */
  double fFront = 0.2;

  /*
   * field of view factor, sums with rest of factors
   */
  double fFieldOfView = 0.0;

  //! agents parameters
  /*
   * agent fov angle
   */

  // /*
  //  * Angle defined when velocity is involved between robot and agent
  //  */
  // double angleMotionDir = 1;

  // /*
  //  * Gaze angle direction, specifically when agent is static
  //  */
  // double angleGazeDir = 1;

  //! parameters for robot field of view
  /*
   * This is the angle of field of view of the robot.
   */

  // double fRobotView = (M_PI - ((M_PI - robotAngleView) * 2));
  double fRobotView = 0.75 * M_PI;

  /*
   * This is the angle of field of view of the robot.
   */
  double robotDistanceView = 6;

  /*
   * This is the velocity that will create de maximum distance for agent evaluation
   */
  double robotVelocityThreshold = 0.38;

  // social costmap data
  nav_msgs::OccupancyGridConstPtr socialCostmap;
  std::vector<int8_t> socialCostmapValues;
};

#endif

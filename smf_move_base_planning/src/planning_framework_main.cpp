/*! \file planning_framework_main.cpp
 * \brief Framework for planning collision-free paths.
 *
 * \date March 5, 2015
 * \author Juan David Hernandez Vega, juandhv@rice.edu
 *
 * \details Framework for planning collision-free paths online. Iterative planning
 * uses last known solution path in order to guarantee that new solution paths are always
 * at least as good as the previous one.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#include <iostream>
#include <vector>

#include <boost/bind.hpp>
#include <math.h>

// standard OMPL
// #include <ompl/control/SpaceInformation.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <planner/RRTstarMod.h>
#include <planner/SSTMod.h>
// ROS
#include <ros/ros.h>
#include <ros/package.h>
// ROS services
#include <std_srvs/Empty.h>
// ROS markers rviz
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
// ROS tf
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
// action server
#include <actionlib/server/simple_action_server.h>

// Planner
#include <new_state_sampler.h>
#include <informed_new_state_sampler.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
#include <state_cost_objective.h>
#include <state_validity_checker_grid_map_R2.h>
#include <local_state_validity_checker_grid_map_SE2.h>
#include "kinematic_diff_model.h"

// smf base controller
#include <nav_msgs/Path.h>
#include <smf_move_base_msgs/Goto2DAction.h>

// pedsim msgs
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

typedef actionlib::SimpleActionServer<smf_move_base_msgs::Goto2DAction>
    SmfBaseGoToActionServer;

//!  OnlinePlannFramework class.
/*!
 * Online Planning Framework.
 * Setup a sampling-based planner for online computation of collision-free paths.
 * C-Space: R2
 * Workspace is represented with Octomaps
 */
class OnlinePlannFramework
{
public:
    //! Constructor
    OnlinePlannFramework();
    //! Planner setup
    void planWithSimpleSetup();
    //! Periodic callback to solve the query.
    void planningTimerCallback();
    //! Callback for getting current vehicle odometry
    void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
    //! Callback for getting the 2D navigation goal
    void queryGoalCallback(const geometry_msgs::PoseStampedConstPtr &nav_goal_msg);
    //! Callback for getting the 2D navigation goal
    void goToActionCallback(const smf_move_base_msgs::Goto2DGoalConstPtr &goto_req);
    //! Procedure to visualize the resulting path
    void visualizeRRT(og::PathGeometric &geopath);
    //! Procedure to visualize the resulting local path
    void visualizeRRTLocal(og::PathGeometric &geopath);
    //! Callback for getting the state of the Smf base controller.
    void controlActiveCallback(const std_msgs::BoolConstPtr &control_active_msg);
    //! check if goal candidate is valid
    bool validateGoalCandidate(const ob::ScopedState<> &goal_candidate);
    //! calculate angle between to points XY
    double calculateAngle(double x1, double y1, double x2, double y2);
    //! get a new valid goal candidate according to the provided goal
    ob::GoalStates *findNewGoalCandidate(const ob::ScopedState<> &goal_candidate);

private:
    // ROS
    ros::NodeHandle nh_, local_nh_;
    ros::Timer timer_;
    ros::Subscriber odom_sub_, nav_goal_sub_, control_active_sub_;
    ros::Publisher solution_path_rviz_pub_, solution_local_path_rviz_pub_, solution_path_control_pub_, query_goal_pose_rviz_pub_, query_goal_radius_rviz_pub_, num_nodes_pub_;

    // ROS action server
    SmfBaseGoToActionServer *goto_action_server_;
    std::string goto_action_;
    smf_move_base_msgs::Goto2DAction goto_action_feedback_;
    smf_move_base_msgs::Goto2DAction goto_action_result_;

    // ROS TF
    tf::Pose last_robot_pose_;
    tf::TransformListener tf_listener_;

    // OMPL, online planner
    og::SimpleSetupPtr simple_setup_global_;
    oc::SimpleSetupPtr simple_setup_local_;
    double timer_period_, solving_time_, xy_goal_tolerance_, local_xy_goal_tolerance_, yaw_goal_tolerance_, robot_base_radius;
    bool opport_collision_check_, reuse_last_best_solution_, local_reuse_last_best_solution_, motion_cost_interpolation_, odom_available_,
        goal_available_, dynamic_bounds_, visualize_tree_,
        control_active_;
    std::vector<double> planning_bounds_x_, planning_bounds_y_, start_state_, goal_map_frame_,
        goal_odom_frame_;
    double goal_radius_, local_goal_radius_, local_path_range_, global_time_percent_, max_trans_vel_, max_rot_vel_;
    std::string planner_name_, local_planner_name_, optimization_objective_, local_optimization_objective_, odometry_topic_, query_goal_topic_,
        solution_path_topic_, world_frame_, octomap_service_, control_active_topic_;
    std::vector<const ob::State *> solution_path_states_, local_solution_path_states_, past_local_solution_path_states_;

    // ====================

    geometry_msgs::Twist current_robot_velocity;
};

//!  Constructor.
/*!
 * Load planner parameters from configuration file.
 * Publishers to visualize the resulting path.
 */
OnlinePlannFramework::OnlinePlannFramework()
    : local_nh_("~"), dynamic_bounds_(false), goto_action_server_(NULL), control_active_(false)
{
    //=======================================================================
    // Get parameters
    //=======================================================================
    planning_bounds_x_.resize(2);
    planning_bounds_y_.resize(2);
    start_state_.resize(2);
    goal_map_frame_.resize(3);
    goal_odom_frame_.resize(3);

    local_nh_.param("world_frame", world_frame_, world_frame_);
    local_nh_.param("planning_bounds_x", planning_bounds_x_, planning_bounds_x_);
    local_nh_.param("planning_bounds_y", planning_bounds_y_, planning_bounds_y_);
    local_nh_.param("start_state", start_state_, start_state_);
    local_nh_.param("goal_state", goal_map_frame_, goal_map_frame_);
    local_nh_.param("timer_period", timer_period_, timer_period_);
    local_nh_.param("solving_time", solving_time_, solving_time_);
    local_nh_.param("opport_collision_check", opport_collision_check_, opport_collision_check_);
    local_nh_.param("planner_name", planner_name_, planner_name_);
    local_nh_.param("reuse_last_best_solution", reuse_last_best_solution_, reuse_last_best_solution_);
    local_nh_.param("local_reuse_last_best_solution", local_reuse_last_best_solution_, local_reuse_last_best_solution_);
    local_nh_.param("optimization_objective", optimization_objective_, optimization_objective_);
    local_nh_.param("motion_cost_interpolation", motion_cost_interpolation_, motion_cost_interpolation_);
    local_nh_.param("odometry_topic", odometry_topic_, odometry_topic_);
    local_nh_.param("query_goal_topic", query_goal_topic_, query_goal_topic_);
    local_nh_.param("goto_action", goto_action_, goto_action_);
    local_nh_.param("solution_path_topic", solution_path_topic_, solution_path_topic_);
    local_nh_.param("dynamic_bounds", dynamic_bounds_, dynamic_bounds_);
    local_nh_.param("xy_goal_tolerance", xy_goal_tolerance_, 0.2);
    local_nh_.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.1);
    local_nh_.param("visualize_tree", visualize_tree_, false);
    local_nh_.param("robot_base_radius", robot_base_radius, robot_base_radius);
    local_nh_.param("local_planner_name", local_planner_name_, local_planner_name_);
    local_nh_.param("local_xy_goal_tolerance", local_xy_goal_tolerance_, local_xy_goal_tolerance_);
    local_nh_.param("local_optimization_objective", local_optimization_objective_, local_optimization_objective_);
    local_nh_.param("local_path_range", local_path_range_, local_path_range_);
    local_nh_.param("global_time_percent", global_time_percent_, global_time_percent_);
    local_nh_.param("max_trans_vel", max_trans_vel_, max_trans_vel_);
    local_nh_.param("max_rot_vel", max_rot_vel_, max_rot_vel_);

    start_state_.resize(3);

    goal_radius_ = xy_goal_tolerance_;
    local_goal_radius_ = local_xy_goal_tolerance_;
    goal_available_ = false;

    //=======================================================================
    // Subscribers
    //=======================================================================
    // Odometry data
    odom_sub_ = nh_.subscribe(odometry_topic_, 1, &OnlinePlannFramework::odomCallback, this);
    odom_available_ = false;

    // 2D Nav Goal
    nav_goal_sub_ = local_nh_.subscribe(query_goal_topic_, 1, &OnlinePlannFramework::queryGoalCallback, this);

    // Controller active flag
    control_active_sub_ =
        local_nh_.subscribe(control_active_topic_, 1, &OnlinePlannFramework::controlActiveCallback, this);

    //=======================================================================
    // Publishers
    //=======================================================================
    solution_path_rviz_pub_ = local_nh_.advertise<visualization_msgs::Marker>("solution_path", 1, true);
    solution_local_path_rviz_pub_ = local_nh_.advertise<visualization_msgs::Marker>("local_solution_path", 1, true);
    solution_path_control_pub_ =
        local_nh_.advertise<nav_msgs::Path>("path", 1, true);
    query_goal_pose_rviz_pub_ =
        local_nh_.advertise<geometry_msgs::PoseStamped>("query_goal_pose_rviz", 1, true);
    query_goal_radius_rviz_pub_ =
        local_nh_.advertise<visualization_msgs::Marker>("query_goal_radius_rviz", 1, true);

    num_nodes_pub_ = local_nh_.advertise<std_msgs::Int32>("smf_num_nodes", 1, true);

    //=======================================================================
    // Action server
    //=======================================================================
    goto_action_server_ = new SmfBaseGoToActionServer(
        ros::NodeHandle(), goto_action_, boost::bind(&OnlinePlannFramework::goToActionCallback, this, _1),
        false);

    //=======================================================================
    // Waiting for odometry
    //=======================================================================
    ros::Rate loop_rate(10);
    while (ros::ok() && !odom_available_)
    {
        ros::spinOnce();
        loop_rate.sleep();
        ROS_WARN("%s:\n\tWaiting for vehicle's odometry\n", ros::this_node::getName().c_str());
    }
    ROS_WARN("%s:\n\tOdometry received\n", ros::this_node::getName().c_str());

    goto_action_server_->start();
}

//! Goto action callback.
/*!
 * Callback for getting the 2D navigation goal
 */
void OnlinePlannFramework::goToActionCallback(const smf_move_base_msgs::Goto2DGoalConstPtr &goto_req)
{
    goal_map_frame_[0] = goto_req->goal.x;
    goal_map_frame_[1] = goto_req->goal.y;
    goal_map_frame_[2] = goto_req->goal.theta;

    double useless_pitch, useless_roll, yaw;

    //=======================================================================
    // Publish RViz Maker
    //=======================================================================
    geometry_msgs::PoseStamped query_goal_msg;
    query_goal_msg.header.frame_id = "map";
    query_goal_msg.header.stamp = ros::Time::now();
    query_goal_msg.pose.position.x = goto_req->goal.x;
    query_goal_msg.pose.position.y = goto_req->goal.y;
    query_goal_msg.pose.position.z = 0.0;
    query_goal_msg.pose.orientation.x = 0.0;
    query_goal_msg.pose.orientation.y = 0.0;
    query_goal_msg.pose.orientation.z = sin(goto_req->goal.theta / 2.0);
    query_goal_msg.pose.orientation.w = cos(goto_req->goal.theta / 2.0);
    query_goal_pose_rviz_pub_.publish(query_goal_msg);

    visualization_msgs::Marker radius_msg;
    radius_msg.header.frame_id = "map";
    radius_msg.header.stamp = ros::Time::now();
    radius_msg.ns = "goal_radius";
    radius_msg.action = visualization_msgs::Marker::ADD;
    radius_msg.pose.orientation.w = 1.0;
    radius_msg.id = 0;
    radius_msg.type = visualization_msgs::Marker::CYLINDER;
    radius_msg.scale.x = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.y = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.z = 0.02;
    radius_msg.color.r = 1.0;
    radius_msg.color.a = 0.5;
    radius_msg.pose.position.x = goto_req->goal.x;
    radius_msg.pose.position.y = goto_req->goal.y;
    radius_msg.pose.position.z = 0.0;
    query_goal_radius_rviz_pub_.publish(radius_msg);

    //=======================================================================
    // Transform from map to odom
    //=======================================================================
    ros::Time t;
    std::string err = "";
    tf::StampedTransform tf_map_to_fixed;
    tf_listener_.getLatestCommonTime("map", "odom", t, &err);
    tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);
    tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

    tf::Point goal_point_odom_frame(goal_map_frame_[0], goal_map_frame_[1], 0.0);
    goal_point_odom_frame = tf_map_to_fixed.inverse() * goal_point_odom_frame;
    goal_odom_frame_[0] = goal_point_odom_frame.getX();
    goal_odom_frame_[1] = goal_point_odom_frame.getY();
    goal_odom_frame_[2] = goal_map_frame_[2] - yaw;

    goal_radius_ = xy_goal_tolerance_;

    //=======================================================================
    // Clean and merge octomap
    //=======================================================================
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;

    // ! COMMENTED TO AVOID UNNEEDED PROCESSING
    // while (nh_.ok() && !ros::service::call("/smf_move_base_mapper/clean_merge_octomap", req, resp))  //
    // {
    //     ROS_WARN("Request to %s failed; trying again...",
    //              nh_.resolveName("/smf_move_base_mapper/clean_merge_octomap").c_str());
    //     usleep(1000000);
    // }
    solution_path_states_.clear();
    local_solution_path_states_.clear();
    goal_available_ = true;

    ros::Rate loop_rate(10);
    while (ros::ok() && (goal_available_ || control_active_))
        loop_rate.sleep();

    smf_move_base_msgs::Goto2DResult result;
    result.success = true;

    goto_action_server_->setSucceeded(result);
}

//! Odometry callback.
/*!
 * Callback for getting updated vehicle odometry
 */
void OnlinePlannFramework::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    if (!odom_available_)
        odom_available_ = true;

    geometry_msgs::Pose predictedPose = odom_msg->pose.pose;

    predictedPose.position.x = odom_msg->pose.pose.position.x;

    predictedPose.position.y = odom_msg->pose.pose.position.y;

    tf::poseMsgToTF(predictedPose, last_robot_pose_);

    double useless_pitch,
        useless_roll, yaw;
    last_robot_pose_.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

    if ((goal_available_) &&
        sqrt(pow(goal_odom_frame_[0] - last_robot_pose_.getOrigin().getX(), 2.0) +
             pow(goal_odom_frame_[1] - last_robot_pose_.getOrigin().getY(), 2.0)) < (goal_radius_ + 0.5))
    {
        goal_available_ = false;
    }

    current_robot_velocity = odom_msg->twist.twist;
}

//! Control active callback.
/*!
 * Callback for getting the state of the Smf base controller
 */
void OnlinePlannFramework::controlActiveCallback(const std_msgs::BoolConstPtr &control_active_msg)
{
    control_active_ = control_active_msg->data;
}

//! Navigation goal callback.
/*!
 * Callback for getting the 2D navigation goal
 */
void OnlinePlannFramework::queryGoalCallback(const geometry_msgs::PoseStampedConstPtr &query_goal_msg)
{
    double useless_pitch, useless_roll, yaw;
    yaw = tf::getYaw(tf::Quaternion(query_goal_msg->pose.orientation.x, query_goal_msg->pose.orientation.y,
                                    query_goal_msg->pose.orientation.z, query_goal_msg->pose.orientation.w));

    goal_map_frame_[0] = query_goal_msg->pose.position.x; // x
    goal_map_frame_[1] = query_goal_msg->pose.position.y; // y
    goal_map_frame_[2] = yaw;

    //=======================================================================
    // Transform from map to odom
    //=======================================================================
    ros::Time t;
    std::string err = "";
    tf::StampedTransform tf_map_to_fixed;
    tf_listener_.getLatestCommonTime("map", "odom", t, &err);
    tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);
    tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

    tf::Point goal_point_odom_frame(goal_map_frame_[0], goal_map_frame_[1], 0.0);
    goal_point_odom_frame = tf_map_to_fixed.inverse() * goal_point_odom_frame;
    goal_odom_frame_[0] = goal_point_odom_frame.getX();
    goal_odom_frame_[1] = goal_point_odom_frame.getY();
    goal_odom_frame_[2] = goal_map_frame_[2] - yaw;

    //=======================================================================
    // Clean and merge octomap
    //=======================================================================
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;
    // ! COMMENTED TO AVOID UNNEEDED PROCESSING
    // while (nh_.ok() && !ros::service::call("/smf_move_base_mapper/clean_merge_octomap", req, resp))  //
    // TODO
    // {
    //     ROS_WARN("Request to %s failed; trying again...",
    //              nh_.resolveName("/smf_move_base_mapper/clean_merge_octomap").c_str());
    //     usleep(1000000);
    // }
    solution_path_states_.clear();
    local_solution_path_states_.clear();
    goal_available_ = true;

    //=======================================================================
    // Publish RViz Maker
    //=======================================================================
    query_goal_pose_rviz_pub_.publish(query_goal_msg);

    visualization_msgs::Marker radius_msg;
    radius_msg.header.frame_id = "map";
    radius_msg.header.stamp = ros::Time::now();
    radius_msg.ns = "goal_radius";
    radius_msg.action = visualization_msgs::Marker::ADD;
    radius_msg.pose.orientation.w = 1.0;
    radius_msg.id = 0;
    radius_msg.type = visualization_msgs::Marker::CYLINDER;
    radius_msg.scale.x = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.y = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.z = 0.02;
    radius_msg.color.r = 1.0;
    radius_msg.color.a = 0.5;
    radius_msg.pose.position.x = goal_map_frame_[0];
    radius_msg.pose.position.y = goal_map_frame_[1];
    radius_msg.pose.position.z = 0.0;
    query_goal_radius_rviz_pub_.publish(radius_msg);
}

//!  Planner setup.
/*!
 * Setup a sampling-based planner using OMPL.
 */
void OnlinePlannFramework::planWithSimpleSetup()
{
    //=======================================================================
    // Instantiate the state space
    //=======================================================================
    ob::StateSpacePtr space = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));

    ob::StateSpacePtr local_space;

    local_space = ob::StateSpacePtr(new ob::SE2StateSpace());

    //=======================================================================
    // Set the bounds for the state space
    //=======================================================================
    ob::RealVectorBounds bounds(2);

    bounds.setLow(0, planning_bounds_x_[0]);
    bounds.setHigh(0, planning_bounds_x_[1]);
    bounds.setLow(1, planning_bounds_y_[0]);
    bounds.setHigh(1, planning_bounds_y_[1]);

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    local_space->as<ob::SE2StateSpace>()->setBounds(bounds);

    //! setup control space

    oc::ControlSpacePtr control_space(new KinDiffControlSpace(local_space));

    ob::RealVectorBounds control_bounds(2);
    control_bounds.setLow(0, 0);
    control_bounds.setHigh(0, max_trans_vel_);
    control_bounds.setLow(1, -max_rot_vel_);
    control_bounds.setHigh(1, max_rot_vel_);

    control_space->as<KinDiffControlSpace>()->setBounds(control_bounds);

    //=======================================================================
    // Define a simple setup class
    //=======================================================================
    simple_setup_global_ = og::SimpleSetupPtr(new og::SimpleSetup(space));
    ob::SpaceInformationPtr si_global = simple_setup_global_->getSpaceInformation();

    // !defining simple setup for local planner
    simple_setup_local_ = oc::SimpleSetupPtr(new oc::SimpleSetup(control_space));
    oc::SpaceInformationPtr si_local = simple_setup_local_->getSpaceInformation();

    // ! ==================================

    si_local->setPropagationStepSize(0.1);

    int _durationMin = 1;
    int _durationMax = 10;
    si_local->setMinMaxControlDuration(_durationMin, _durationMax);

    //=======================================================================
    // Create a planner for the defined space
    //=======================================================================
    ob::PlannerPtr planner;
    if (planner_name_.compare("RRT") == 0)
        planner = ob::PlannerPtr(new og::RRT(si_global));
    else if (planner_name_.compare("PRMstar") == 0)
        planner = ob::PlannerPtr(new og::PRMstar(si_global));
    else if (planner_name_.compare("RRTstar") == 0)
        planner = ob::PlannerPtr(new og::RRTstar(si_global));
    else if (planner_name_.compare("RRTstarMod") == 0)
        planner = ob::PlannerPtr(new og::RRTstarMod(si_global));
    else
        planner = ob::PlannerPtr(new og::RRTstar(si_global));

    // !LOCAL PLANNER SETUP
    ob::PlannerPtr local_planner;
    if (local_planner_name_.compare("RRT") == 0)
        local_planner = ob::PlannerPtr(new oc::RRT(si_local));
    else
    {
        local_planner = ob::PlannerPtr(new oc::SST(si_local));
        local_planner->as<oc::SST>()->setSelectionRadius(0.05);
        local_planner->as<oc::SST>()->setPruningRadius(0.001);
    }

    //=======================================================================
    // Set the setup planner
    //=======================================================================
    simple_setup_global_->setPlanner(planner);

    simple_setup_local_->setPlanner(local_planner);

    //=======================================================================
    // Create a start and goal states
    //=======================================================================
    double useless_pitch, useless_roll, yaw;
    last_robot_pose_.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);
    start_state_[0] = double(last_robot_pose_.getOrigin().getX() + double(current_robot_velocity.linear.x * (solving_time_ + 0.15))); // x
    start_state_[1] = double(last_robot_pose_.getOrigin().getY() + double(current_robot_velocity.linear.y * (solving_time_ + 0.15))); // y
    start_state_[2] = double(yaw);

    // create a start state
    //! GLOBAL START STATE
    ob::ScopedState<> start(space);
    start[0] = double(start_state_[0]); // x
    start[1] = double(start_state_[1]); // y

    //! LOCAL START STATE
    ob::ScopedState<> local_start(local_space);
    local_start[0] = double(start_state_[0]); // x
    local_start[1] = double(start_state_[1]); // y
    local_start[2] = double(start_state_[2]); // yaw

    // create a goal state
    //! GLOBAL GOAL STATE
    ob::ScopedState<> goal(space);
    goal[0] = double(goal_map_frame_[0]); // x
    goal[1] = double(goal_map_frame_[1]); // y

    //! LOCAL GOAL STATE
    ob::ScopedState<> local_goal(local_space);
    local_goal[0] = double(goal_map_frame_[0]); // x
    local_goal[1] = double(goal_map_frame_[1]); // y
    local_goal[2] = double(goal_map_frame_[2]); // yaw

    //=======================================================================
    // Set the start and goal states
    //=======================================================================
    simple_setup_global_->setStartState(start);
    simple_setup_global_->setGoalState(goal, goal_radius_);
    // simple_setup_->getStateSpace()->setValidSegmentCountFactor(5.0);

    // !LOCAL GOAL SETUP
    simple_setup_local_->setStartState(local_start);
    simple_setup_local_->setGoalState(local_goal, local_goal_radius_);

    //=======================================================================
    // Set state validity checking for this space
    //=======================================================================
    ob::StateValidityCheckerPtr om_stat_val_check;
    om_stat_val_check = ob::StateValidityCheckerPtr(
        new GridMapStateValidityCheckerR2(simple_setup_global_->getSpaceInformation(), opport_collision_check_,
                                          planning_bounds_x_, planning_bounds_y_));
    simple_setup_global_->setStateValidityChecker(om_stat_val_check);

    // !VALIDITY CHECKING FOR LOCAL PLANNER
    ob::StateValidityCheckerPtr local_om_stat_val_check;
    local_om_stat_val_check = ob::StateValidityCheckerPtr(
        new LocalGridMapStateValidityCheckerSE2(simple_setup_local_->getSpaceInformation(), opport_collision_check_,
                                                planning_bounds_x_, planning_bounds_y_));

    simple_setup_local_->setStateValidityChecker(local_om_stat_val_check);

    simple_setup_local_->setStatePropagator(oc::StatePropagatorPtr(new KinDiffStatePropagator(simple_setup_local_->getSpaceInformation())));

    //=======================================================================
    // Set optimization objective
    //=======================================================================
    if (optimization_objective_.compare("PathLength") == 0) // path length Objective
        simple_setup_global_->getProblemDefinition()->setOptimizationObjective(getPathLengthObjective(si_global));
    else if (optimization_objective_.compare("SocialHeatmap") == 0) // Social Costmap
        simple_setup_global_->getProblemDefinition()->setOptimizationObjective(
            getSocialHeatmapObjective(si_global, motion_cost_interpolation_));
    else
        simple_setup_global_->getProblemDefinition()->setOptimizationObjective(getPathLengthObjective(si_global));

    // !OPTIMIZATION OBJECTIVE FOR LOCAL PLANNER

    std::vector<const ob::State *> dummy_global_path_feedback;

    if (local_optimization_objective_.compare("PathLength") == 0) // path length Objective
        simple_setup_local_->getProblemDefinition()->setOptimizationObjective(getPathLengthObjective(si_local));
    else if (local_optimization_objective_.compare("SocialComfort") == 0) // Social Comfort
        simple_setup_local_->getProblemDefinition()->setOptimizationObjective(
            getSocialComfortObjective(si_local, motion_cost_interpolation_, local_space, simple_setup_local_->getPlanner(), dummy_global_path_feedback));
    else if (local_optimization_objective_.compare("SocialHeatmap") == 0) // Social Costmap
        simple_setup_local_->getProblemDefinition()->setOptimizationObjective(
            getSocialHeatmapObjective(si_local, motion_cost_interpolation_));
    else
        simple_setup_local_->getProblemDefinition()->setOptimizationObjective(getPathLengthObjective(si_local));

    //=======================================================================
    // Perform setup steps for the planner
    //=======================================================================
    simple_setup_global_->setup();

    simple_setup_local_->setup();

    static_cast<KinDiffStatePropagator *>(simple_setup_local_->getStatePropagator().get())->setIntegrationTimeStep(simple_setup_local_->getSpaceInformation()->getPropagationStepSize());

    //=======================================================================
    // Print information
    //=======================================================================
    // planner->printProperties(//std::cout);// print planner properties
    // si->printSettings(//std::cout);// print the settings for this space

    //=======================================================================
    // Activate a timer for incremental planning
    //=======================================================================
    //	timer_ = nh_.createTimer(ros::Duration(timer_period_), &OnlinePlannFramework::planningTimerCallback,
    // this);
    //
    //	ros::spin();
    ros::Rate loop_rate(1 / (timer_period_ - solving_time_)); // 10 hz
    // goal_available_ = true;

    //	ros::AsyncSpinner spinner(4); // Use 4 threads
    //	spinner.start();
    // ros::waitForShutdown();
    while (ros::ok())
    {
        if (goal_available_)
            ROS_INFO("%s: goal available", ros::this_node::getName().c_str());
        OnlinePlannFramework::planningTimerCallback();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

//!  Periodic callback to solve the query.
/*!
 * Solve the query.
 */
void OnlinePlannFramework::planningTimerCallback()
{
    if (goal_available_)
    {
        //=======================================================================
        // Transform from map to odom
        //=======================================================================
        double useless_pitch, useless_roll, yaw;
        ros::Time t;
        std::string err = "";
        tf::StampedTransform tf_map_to_fixed;
        tf_listener_.getLatestCommonTime("map", "odom", t, &err);
        tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);
        tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

        tf::Point goal_point_odom_frame(goal_map_frame_[0], goal_map_frame_[1], 0.0);
        goal_point_odom_frame = tf_map_to_fixed.inverse() * goal_point_odom_frame;
        goal_odom_frame_[0] = goal_point_odom_frame.getX();
        goal_odom_frame_[1] = goal_point_odom_frame.getY();
        goal_odom_frame_[2] = goal_map_frame_[2] - yaw;

        if (dynamic_bounds_)
        {
            //=======================================================================
            // Set the bounds for the state space
            //=======================================================================
            ob::RealVectorBounds bounds(2);

            if (last_robot_pose_.getOrigin().getX() < goal_odom_frame_[0])
            {
                if (last_robot_pose_.getOrigin().getX() - 10.0 < planning_bounds_x_[0])
                    bounds.setLow(0, planning_bounds_x_[0]);
                else
                    bounds.setLow(0, last_robot_pose_.getOrigin().getX() - 10.0);

                if (goal_odom_frame_[0] + 5.0 > planning_bounds_x_[1])
                    bounds.setHigh(0, planning_bounds_x_[1]);
                else
                    bounds.setHigh(0, goal_odom_frame_[0] + 10.0);
            }
            else
            {
                if (last_robot_pose_.getOrigin().getX() + 10.0 > planning_bounds_x_[1])
                    bounds.setHigh(0, planning_bounds_x_[1]);
                else
                    bounds.setHigh(0, last_robot_pose_.getOrigin().getX() + 10.0);

                if (goal_odom_frame_[0] - 10.0 < planning_bounds_x_[0])
                    bounds.setLow(0, planning_bounds_x_[0]);
                else
                    bounds.setLow(0, goal_odom_frame_[0] - 10.0);
            }

            if (last_robot_pose_.getOrigin().getY() < goal_odom_frame_[1])
            {
                if (last_robot_pose_.getOrigin().getY() - 10.0 < planning_bounds_y_[0])
                    bounds.setLow(1, planning_bounds_y_[0]);
                else
                    bounds.setLow(1, last_robot_pose_.getOrigin().getY() - 10.0);

                if (goal_odom_frame_[1] + 10.0 > planning_bounds_y_[1])
                    bounds.setHigh(1, planning_bounds_y_[1]);
                else
                    bounds.setHigh(1, goal_odom_frame_[1] + 10.0);
            }
            else
            {
                if (last_robot_pose_.getOrigin().getY() + 10.0 > planning_bounds_y_[1])
                    bounds.setHigh(1, planning_bounds_y_[1]);
                else
                    bounds.setHigh(1, last_robot_pose_.getOrigin().getY() + 10.0);

                if (goal_odom_frame_[1] - 10.0 < planning_bounds_y_[0])
                    bounds.setLow(1, planning_bounds_y_[0]);
                else
                    bounds.setLow(1, goal_odom_frame_[1] - 10.0);
            }

            simple_setup_global_->getStateSpace()->as<ob::RealVectorStateSpace>()->setBounds(bounds);

            simple_setup_local_->getStateSpace()->as<ob::SE2StateSpace>()->setBounds(bounds);
        }
        //=======================================================================
        // Set new start state
        //=======================================================================

        ob::ScopedState<> start(simple_setup_global_->getSpaceInformation()->getStateSpace());
        ob::ScopedState<> goal(simple_setup_global_->getSpaceInformation()->getStateSpace());

        last_robot_pose_.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

        start[0] = double(last_robot_pose_.getOrigin().getX() + double(current_robot_velocity.linear.x * (solving_time_ + 0.15))); // x
        start[1] = double(last_robot_pose_.getOrigin().getY() + double(current_robot_velocity.linear.y * (solving_time_ + 0.15))); // y

        goal[0] = double(goal_odom_frame_[0]); // x
        goal[1] = double(goal_odom_frame_[1]); // y

        //======================================================================
        // Set the start and goal states
        //=======================================================================
        simple_setup_global_->clear();
        simple_setup_global_->clearStartStates();
        simple_setup_global_->setStartState(start);
        simple_setup_global_->setGoalState(goal, goal_radius_);
        //
        simple_setup_global_->getStateSpace()->setValidSegmentCountFactor(15.0);

        // local_solution_path_states_.clear();

        // !==================================

        //=======================================================================
        // Set a modified sampler
        //=======================================================================
        if (reuse_last_best_solution_)
            simple_setup_global_->getSpaceInformation()
                ->getStateSpace()
                ->setStateSamplerAllocator(
                    std::bind(newAllocStateSampler, std::placeholders::_1, simple_setup_global_->getPlanner(),
                              solution_path_states_));

        //=======================================================================
        // Set state validity checking for this space
        //=======================================================================
        ob::StateValidityCheckerPtr om_stat_val_check;
        om_stat_val_check = ob::StateValidityCheckerPtr(
            new GridMapStateValidityCheckerR2(simple_setup_global_->getSpaceInformation(), opport_collision_check_,
                                              planning_bounds_x_, planning_bounds_y_));
        simple_setup_global_->setStateValidityChecker(om_stat_val_check);

        //=======================================================================
        // Set optimization objective
        //=======================================================================
        if (optimization_objective_.compare("PathLength") == 0) // path length Objective
            simple_setup_global_->getProblemDefinition()->setOptimizationObjective(
                getPathLengthObjective(simple_setup_global_->getSpaceInformation()));
        else if (optimization_objective_.compare("SocialHeatmap") == 0) // Social Costmap
            simple_setup_global_->getProblemDefinition()->setOptimizationObjective(
                getSocialHeatmapObjective(simple_setup_global_->getSpaceInformation(), motion_cost_interpolation_));
        else
            simple_setup_global_->getProblemDefinition()->setOptimizationObjective(
                getPathLengthObjective(simple_setup_global_->getSpaceInformation()));

        // ! SIMPLE SETUP LOCAL

        simple_setup_local_->clearStartStates();
        ob::ScopedState<> local_start(simple_setup_local_->getSpaceInformation()->getStateSpace());
        ob::ScopedState<> local_goal(simple_setup_local_->getSpaceInformation()->getStateSpace());

        //
        simple_setup_local_->getStateSpace()->setValidSegmentCountFactor(15.0);

        //=======================================================================
        // Set state validity checking for this space
        //=======================================================================
        ob::StateValidityCheckerPtr local_om_stat_val_check;
        local_om_stat_val_check = ob::StateValidityCheckerPtr(
            new LocalGridMapStateValidityCheckerSE2(simple_setup_local_->getSpaceInformation(), opport_collision_check_,
                                                    planning_bounds_x_, planning_bounds_y_));
        simple_setup_local_->setStateValidityChecker(local_om_stat_val_check);

        //=======================================================================
        // Attempt to solve the problem within one second of planning time
        //=======================================================================
        ob::PlannerStatus solved = simple_setup_global_->solve(solving_time_ * (global_time_percent_ / 100));

        bool solution_found = false;
        bool use_last_local_path = false;

        if (solved && simple_setup_global_->haveExactSolutionPath())
        {

            solution_found = true;
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path

            og::PathGeometric path = simple_setup_global_->getSolutionPath();

            // generates varios little segments for the waypoints obtained from the planner
            path.interpolate(int(path.length() / 0.5));

            // path_planning_msgs::PathConstSpeed solution_path;
            ROS_INFO("%s:\n\tpath with cost %f has been found with simple_setup\n",
                     ros::this_node::getName().c_str(),
                     path.cost(simple_setup_global_->getProblemDefinition()->getOptimizationObjective()).value());

            std::vector<ob::State *> path_states;
            path_states = path.getStates();

            double distance_to_goal =
                sqrt(pow(goal_odom_frame_[0] - path_states[path_states.size() - 1]
                                                   ->as<ob::RealVectorStateSpace::StateType>()
                                                   ->values[0],
                         2.0) +
                     pow(goal_odom_frame_[1] - path_states[path_states.size() - 1]
                                                   ->as<ob::RealVectorStateSpace::StateType>()
                                                   ->values[1],
                         2.0));

            if (simple_setup_global_->haveExactSolutionPath() || distance_to_goal <= goal_radius_)
            {

                if (reuse_last_best_solution_)
                {
                    ob::StateSpacePtr space = simple_setup_global_->getStateSpace();
                    solution_path_states_.clear();
                    for (int i = path_states.size() - 1; i >= 0; i--)
                    {
                        ob::State *s = space->allocState();
                        space->copyState(s, path_states[i]);
                        solution_path_states_.push_back(s);
                    }
                }

                // ! GLOBAL PATH VISUALIZE
                visualizeRRT(path);
                // ======================================================================

                //=======================================================================
                // ! LOCAL PLANNER SOLVE
                //=======================================================================

                std::vector<const ob::State *> global_path_feedback;
                ob::StateSpacePtr local_space = simple_setup_local_->getStateSpace();

                double local_path_distance = 0;

                local_goal[0] = double(solution_path_states_[0]->as<ob::RealVectorStateSpace::StateType>()->values[0]); // x
                local_goal[1] = double(solution_path_states_[0]->as<ob::RealVectorStateSpace::StateType>()->values[1]); // y
                local_goal[2] = double(0);

                ob::SpaceInformationPtr si_global = simple_setup_global_->getSpaceInformation();

                for (int i = solution_path_states_.size() - 1; i >= 1; i--)
                {

                    local_path_distance += si_global->distance(solution_path_states_[i - 1]->as<ob::RealVectorStateSpace::StateType>(), solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>());

                    ob::State *s = local_space->allocState();

                    s->as<ob::SE2StateSpace::StateType>()->setX(solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[0]);
                    s->as<ob::SE2StateSpace::StateType>()->setY(solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[1]);

                    double state_angle = calculateAngle(solution_path_states_[i - 1]->as<ob::RealVectorStateSpace::StateType>()->values[1],
                                                        solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[1],
                                                        solution_path_states_[i - 1]->as<ob::RealVectorStateSpace::StateType>()->values[0],
                                                        solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[0]);

                    s->as<ob::SE2StateSpace::StateType>()->setYaw(state_angle);

                    global_path_feedback.push_back(s);

                    if (local_path_distance >= local_path_range_)
                    {
                        local_goal[0] = double(solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[0]); // x
                        local_goal[1] = double(solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[1]); // y
                        local_goal[2] = double(state_angle);
                        break;
                    }
                }

                // adding last local solution states
                if (local_reuse_last_best_solution_)
                {
                    for (int i = 0; i < local_solution_path_states_.size(); i++)
                    {
                        ob::State *s = local_space->allocState();
                        local_space->copyState(s, local_solution_path_states_[i]);
                        global_path_feedback.push_back(s);
                    }
                }

                std::reverse(global_path_feedback.begin(), global_path_feedback.end());

                //======================================================================
                // Set the start and goal states
                //=======================================================================

                if (validateGoalCandidate(local_goal))
                {

                    double waypoints_diff_x;
                    double waypoints_diff_y;

                    waypoints_diff_x = abs(global_path_feedback[global_path_feedback.size() - 1]->as<ob::SE2StateSpace::StateType>()->getX() - goal[0]);
                    waypoints_diff_y = abs(global_path_feedback[global_path_feedback.size() - 1]->as<ob::SE2StateSpace::StateType>()->getY() - goal[1]);

                    if ((waypoints_diff_x < 3 && waypoints_diff_y < 3))
                    {

                        local_goal[0] = double(solution_path_states_[0]->as<ob::RealVectorStateSpace::StateType>()->values[0]); // x
                        local_goal[1] = double(solution_path_states_[0]->as<ob::RealVectorStateSpace::StateType>()->values[1]); // y
                        local_goal[2] = double(goal_map_frame_[2]);                                                             // yaw

                        simple_setup_local_->setGoalState(local_goal, goal_radius_);
                    }
                    else
                    {
                        simple_setup_local_->setGoalState(local_goal, local_goal_radius_);
                    }
                }
                else
                {

                    ob::GoalStates *new_goal_local = findNewGoalCandidate(local_goal);
                    simple_setup_local_->setGoal(ob::GoalPtr(new_goal_local));
                }

                //=======================================================================
                // Set a modified sampler
                //=======================================================================

                if (reuse_last_best_solution_)
                {

                    //=======================================================================
                    // Set optimization objective
                    //=======================================================================
                    if (local_optimization_objective_.compare("PathLength") == 0) // path length Objective
                        simple_setup_local_->getProblemDefinition()->setOptimizationObjective(
                            getPathLengthObjective(simple_setup_local_->getSpaceInformation()));
                    else if (local_optimization_objective_.compare("SocialComfort") == 0)
                        simple_setup_local_->getProblemDefinition()->setOptimizationObjective(
                            getSocialComfortObjective(simple_setup_local_->getSpaceInformation(), motion_cost_interpolation_, local_space, simple_setup_local_->getPlanner(), global_path_feedback));
                    else if (local_optimization_objective_.compare("SocialHeatmap") == 0) // Social Costmap
                        simple_setup_local_->getProblemDefinition()->setOptimizationObjective(
                            getSocialHeatmapObjective(simple_setup_local_->getSpaceInformation(), motion_cost_interpolation_));
                    else
                        simple_setup_local_->getProblemDefinition()->setOptimizationObjective(
                            getPathLengthObjective(simple_setup_local_->getSpaceInformation()));
                }

                oc::SpaceInformationPtr si_local = simple_setup_local_->getSpaceInformation();

                si_local->setPropagationStepSize(0.1);

                int _durationMin = 1;
                int _durationMax = 10;
                si_local->setMinMaxControlDuration(_durationMin, _durationMax);

                // setting start states in planner

                ob::PlannerPtr local_planner;
                if (local_planner_name_.compare("RRT") == 0)
                    local_planner = ob::PlannerPtr(new oc::RRT(si_local));
                else
                {
                    local_planner = ob::PlannerPtr(new oc::SST(si_local));
                    local_planner->as<oc::SST>()->setSelectionRadius(0.05);
                    local_planner->as<oc::SST>()->setPruningRadius(0.001);
                }

                // ! check if last path is collision free
                bool is_past_path_free = false;
                double distance_to_last_point = 0;
                bool is_past_path_in_line = false;
                double max_distance_tolerance = 2;

                og::PathGeometric past_local_path = og::PathGeometric(simple_setup_local_->getSpaceInformation(), past_local_solution_path_states_);

                // CHECK IF LAST LOCAL PATH IS FREE
                if (past_local_solution_path_states_.size() > 0)
                {
                    is_past_path_free = past_local_path.check();
                }

                // CHECK DISTANCE FROM LAST NODE IN LOCAL PATH TO NEAREST NODE IN GLOBAL PATH
                if (is_past_path_free)
                {

                    ob::ScopedState<> local_state(simple_setup_global_->getSpaceInformation()->getStateSpace());
                    local_state[0] = past_local_solution_path_states_[0]->as<ob::SE2StateSpace::StateType>()->getX();
                    local_state[1] = past_local_solution_path_states_[0]->as<ob::SE2StateSpace::StateType>()->getY();

                    int nearest_node = path.getClosestIndex(local_state->as<ob::RealVectorStateSpace::StateType>());

                    double distance_nearest_node = si_global->distance(path_states[nearest_node]->as<ob::RealVectorStateSpace::StateType>(), local_state->as<ob::RealVectorStateSpace::StateType>());

                    if (distance_nearest_node < max_distance_tolerance)
                    {
                        ob::State *s = local_space->allocState();
                        s->as<ob::SE2StateSpace::StateType>()->setX(path_states[nearest_node]->as<ob::RealVectorStateSpace::StateType>()->values[0]);
                        s->as<ob::SE2StateSpace::StateType>()->setY(path_states[nearest_node]->as<ob::RealVectorStateSpace::StateType>()->values[1]);
                        if (simple_setup_local_->getSpaceInformation()->checkMotion(s, past_local_solution_path_states_[0]->as<ob::SE2StateSpace::StateType>()))
                        {
                            is_past_path_in_line = true;
                        }
                    }
                }

                // ==================================

                // !ESTIMATION OF START POINT PROJECTED ON THE LOCAL PATH
                // #########################################################

                nav_msgs::OdometryConstPtr odom_data = ros::topic::waitForMessage<nav_msgs::Odometry>(odometry_topic_);

                tf::Quaternion q(
                    odom_data->pose.pose.orientation.x,
                    odom_data->pose.pose.orientation.y,
                    odom_data->pose.pose.orientation.z,
                    odom_data->pose.pose.orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                local_start[0] = double(odom_data->pose.pose.position.x); // x
                local_start[1] = double(odom_data->pose.pose.position.y); // y
                local_start[2] = double(yaw);                             // yaw

                // ! calculate distance from robot position to last past local path waypoint
                if (is_past_path_in_line)
                {
                    distance_to_last_point = std::sqrt(std::pow(odom_data->pose.pose.position.x - past_local_solution_path_states_[0]->as<ob::SE2StateSpace::StateType>()->getX(), 2) + std::pow(odom_data->pose.pose.position.y - past_local_solution_path_states_[0]->as<ob::SE2StateSpace::StateType>()->getY(), 2));
                }

                // ========================================

                simple_setup_local_->setStartState(local_start);

                // !###############################################################

                //=======================================================================
                // Set the setup planner
                //=======================================================================

                simple_setup_local_->setPlanner(local_planner);

                //=======================================================================
                // Attempt to solve the problem within one second of planning time
                //=======================================================================
                ob::PlannerStatus solved_local = simple_setup_local_->solve(solving_time_ * (1 - (global_time_percent_ / 100)));

                if (solved_local && simple_setup_local_->haveExactSolutionPath())
                {
                    solution_found = true;
                    // get the goal representation from the problem definition (not the same as the goal state)
                    // and inquire about the found path

                    og::PathGeometric path_local = simple_setup_local_->getSolutionPath().asGeometric();

                    // generates varios little segments for the waypoints obtained from the planner
                    path_local.interpolate(int(path_local.length() / 0.2));

                    // path_planning_msgs::PathConstSpeed solution_path;
                    ROS_INFO("%s:\n\tlocal path with cost %f has been found with simple_setup\n",
                             ros::this_node::getName().c_str(),
                             path_local.cost(simple_setup_local_->getProblemDefinition()->getOptimizationObjective()).value());

                    if (distance_to_last_point >= 1.0 && past_local_path.cost(simple_setup_local_->getProblemDefinition()->getOptimizationObjective()).value() <= path_local.cost(simple_setup_local_->getProblemDefinition()->getOptimizationObjective()).value() && past_local_path.smoothness() <= path_local.smoothness())
                    {
                        ROS_INFO("%s:\n\tusing past local solution.\n",
                                 ros::this_node::getName().c_str());
                        use_last_local_path = true;
                    }

                    if (!use_last_local_path)
                    {
                        std::vector<ob::State *> local_path_states;
                        local_path_states = path_local.getStates();

                        double distance_to_goal = sqrt(pow(local_goal[0] - local_path_states[local_path_states.size() - 1]
                                                                               ->as<ob::SE2StateSpace::StateType>()
                                                                               ->getX(),
                                                           2.0) +
                                                       pow(local_goal[1] - local_path_states[local_path_states.size() - 1]
                                                                               ->as<ob::SE2StateSpace::StateType>()
                                                                               ->getY(),
                                                           2.0));

                        std::vector<ob::State *> controller_local_path_states;

                        if (simple_setup_local_->haveExactSolutionPath() || distance_to_goal <= local_goal_radius_)
                        {
                            // ======================================================================
                            ob::StateSpacePtr space = simple_setup_local_->getStateSpace();

                            ob::ScopedState<> current_robot_state(simple_setup_local_->getSpaceInformation()->getStateSpace());
                            current_robot_state[0] = odom_data->pose.pose.position.x;
                            current_robot_state[1] = odom_data->pose.pose.position.y;

                            for (int i = 0; i < local_path_states.size(); i++)
                            {
                                ob::State *s = space->allocState();
                                space->copyState(s, local_path_states[i]);
                                controller_local_path_states.push_back(s);
                            }

                            std::vector<ob::State *> controller_path_feedback_states = controller_local_path_states;

                            std::reverse(controller_path_feedback_states.begin(), controller_path_feedback_states.end());

                            og::PathGeometric path_local_visual = og::PathGeometric(simple_setup_local_->getSpaceInformation());

                            for (int i = 0; i < controller_path_feedback_states.size(); i++)
                            {
                                path_local_visual.append(controller_path_feedback_states[i]);
                            }

                            visualizeRRTLocal(path_local_visual);
                            // =============================================================================

                            if (reuse_last_best_solution_)
                            {
                                local_solution_path_states_.clear();
                                past_local_solution_path_states_.clear();
                                // solution_path_states_.reserve(path_states.size());
                                for (int i = local_path_states.size() - 1; i >= 0; i--)
                                {
                                    ob::State *s = space->allocState();
                                    space->copyState(s, local_path_states[i]);
                                    local_solution_path_states_.push_back(s);
                                    past_local_solution_path_states_.push_back(s);
                                }
                            }
                        }

                        // !end of local planner solve

                        //=======================================================================
                        // Controller
                        //=======================================================================
                        if (controller_local_path_states.size() > 0)
                        {
                            nav_msgs::Path solution_path_for_control;
                            for (unsigned int i = 0; i < controller_local_path_states.size(); i++)
                            {
                                geometry_msgs::PoseStamped p;

                                p.pose.position.x = controller_local_path_states[i]->as<ob::SE2StateSpace::StateType>()->getX();
                                p.pose.position.y = controller_local_path_states[i]->as<ob::SE2StateSpace::StateType>()->getY();

                                if (i == (controller_local_path_states.size() - 1))
                                {
                                    if (goal_available_)
                                    {
                                        ros::Time t;
                                        std::string err = "";
                                        tf::StampedTransform tf_map_to_fixed;
                                        tf_listener_.getLatestCommonTime("map", "odom", t, &err);
                                        tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);

                                        tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

                                        tf2::Quaternion myQuaternion;

                                        myQuaternion.setRPY(useless_roll, useless_pitch, goal_map_frame_[2] - yaw);

                                        myQuaternion = myQuaternion.normalize();

                                        p.pose.orientation.x = myQuaternion.getX();
                                        p.pose.orientation.y = myQuaternion.getY();
                                        p.pose.orientation.z = myQuaternion.getZ();
                                        p.pose.orientation.w = myQuaternion.getW();
                                    }
                                }
                                solution_path_for_control.poses.push_back(p);
                            }
                            // ROS_INFO_STREAM("complete path: " << solution_path_for_control);
                            solution_path_control_pub_.publish(solution_path_for_control);
                        }
                    }
                }
                else
                {
                    solution_found = false;
                }
                //=======================================================================
                // Clear previous solution path
                //=======================================================================
                simple_setup_global_->clear();
                simple_setup_local_->clear();
            }
        }
        else
        {
            // visualizeRRTTree();

            solution_found = false;
        }

        if (!solution_found || use_last_local_path)
        {
            ROS_WARN("%s:\n\tpath has not been found\n", ros::this_node::getName().c_str());

            simple_setup_local_->clear();

            if (past_local_solution_path_states_.size() > 0)
            {
                std::vector<const ob::State *> local_solution_path_states_copy_;

                ob::StateSpacePtr local_space = simple_setup_local_->getStateSpace();

                ob::ScopedState<> current_robot_state(simple_setup_local_->getSpaceInformation()->getStateSpace());
                current_robot_state[0] = last_robot_pose_.getOrigin().getX();
                current_robot_state[1] = last_robot_pose_.getOrigin().getY();

                oc::SpaceInformationPtr si_local = simple_setup_local_->getSpaceInformation();

                og::PathGeometric last_path = og::PathGeometric(simple_setup_local_->getSpaceInformation(), past_local_solution_path_states_);

                int closest_index = last_path.getClosestIndex(current_robot_state->as<ob::State>());

                for (int i = 0; i < closest_index; i++)
                {
                    ob::State *s = local_space->allocState();
                    local_space->copyState(s, past_local_solution_path_states_[i]);
                    local_solution_path_states_copy_.push_back(s);
                }

                std::reverse(local_solution_path_states_copy_.begin(), local_solution_path_states_copy_.end());
                ROS_INFO("%s:\n\tsending partial last possible path\n", ros::this_node::getName().c_str());
                nav_msgs::Path solution_path_for_control;
                og::PathGeometric path_visualize = og::PathGeometric(simple_setup_local_->getSpaceInformation());

                // adding first waypoint
                if (simple_setup_local_->getStateValidityChecker()->isValid(local_solution_path_states_copy_[0]))
                {
                    // ROS_INFO("%s:\n\tadding first waypoint\n", ros::this_node::getName().c_str());

                    geometry_msgs::PoseStamped p;
                    p.pose.position.x = local_solution_path_states_copy_[0]->as<ob::SE2StateSpace::StateType>()->getX();
                    p.pose.position.y = local_solution_path_states_copy_[0]->as<ob::SE2StateSpace::StateType>()->getY();

                    if (0 == (local_solution_path_states_copy_.size() - 1))
                    {
                        if (goal_available_)
                        {
                            ros::Time t;
                            std::string err = "";
                            tf::StampedTransform tf_map_to_fixed;
                            tf_listener_.getLatestCommonTime("map", "odom", t, &err);
                            tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);

                            tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

                            tf2::Quaternion myQuaternion;

                            myQuaternion.setRPY(useless_roll, useless_pitch, goal_map_frame_[2] - yaw);

                            myQuaternion = myQuaternion.normalize();

                            p.pose.orientation.x = myQuaternion.getX();
                            p.pose.orientation.y = myQuaternion.getY();
                            p.pose.orientation.z = myQuaternion.getZ();
                            p.pose.orientation.w = myQuaternion.getW();
                        }
                    }
                    solution_path_for_control.poses.push_back(p);
                    path_visualize.append(local_solution_path_states_copy_[0]);
                }

                // adding rest of nodes
                bool lastNode = false;

                for (unsigned int i = 0; (i < local_solution_path_states_copy_.size() - 1) && (!lastNode); i++)
                {
                    if (simple_setup_local_->getSpaceInformation()->checkMotion(local_solution_path_states_copy_[i],
                                                                                local_solution_path_states_copy_[i + 1]) ||
                        (local_solution_path_states_copy_.size() > 3 && i < 3))
                    {
                        // ROS_INFO("%s:\n\tadding possible waypoint\n", ros::this_node::getName().c_str());

                        geometry_msgs::PoseStamped p;

                        p.pose.position.x = local_solution_path_states_copy_[i + 1]
                                                ->as<ob::SE2StateSpace::StateType>()
                                                ->getX();
                        p.pose.position.y = local_solution_path_states_copy_[i + 1]
                                                ->as<ob::SE2StateSpace::StateType>()
                                                ->getY();

                        if (i == (local_solution_path_states_copy_.size() - 1))
                        {
                            if (goal_available_)
                            {
                                ros::Time t;
                                std::string err = "";
                                tf::StampedTransform tf_map_to_fixed;
                                tf_listener_.getLatestCommonTime("map", "odom", t, &err);
                                tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);

                                tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

                                tf2::Quaternion myQuaternion;

                                myQuaternion.setRPY(useless_roll, useless_pitch, goal_map_frame_[2] - yaw);

                                myQuaternion = myQuaternion.normalize();

                                p.pose.orientation.x = myQuaternion.getX();
                                p.pose.orientation.y = myQuaternion.getY();
                                p.pose.orientation.z = myQuaternion.getZ();
                                p.pose.orientation.w = myQuaternion.getW();
                            }
                        }
                        solution_path_for_control.poses.push_back(p);
                        path_visualize.append(local_solution_path_states_copy_[i + 1]);
                    }
                    else
                    {
                        // ROS_INFO("%s:\n\tfound not possible motion\n", ros::this_node::getName().c_str());

                        double angle;

                        angle = atan2(local_solution_path_states_copy_[i + 1]
                                              ->as<ob::SE2StateSpace::StateType>()
                                              ->getY() -
                                          local_solution_path_states_copy_[i]
                                              ->as<ob::SE2StateSpace::StateType>()
                                              ->getY(),
                                      local_solution_path_states_copy_[i + 1]
                                              ->as<ob::SE2StateSpace::StateType>()
                                              ->getX() -
                                          local_solution_path_states_copy_[i]
                                              ->as<ob::SE2StateSpace::StateType>()
                                              ->getX());

                        int counter = 1;
                        while (!lastNode)
                        {
                            ob::ScopedState<> posEv(simple_setup_local_->getStateSpace());

                            posEv[0] = double(local_solution_path_states_copy_[i]
                                                  ->as<ob::SE2StateSpace::StateType>()
                                                  ->getX() +
                                              counter * robot_base_radius * std::cos(angle)); // x
                            posEv[1] = double(local_solution_path_states_copy_[i]
                                                  ->as<ob::SE2StateSpace::StateType>()
                                                  ->getY() +
                                              counter * robot_base_radius * std::sin(angle)); // y

                            if (!simple_setup_local_->getSpaceInformation()->checkMotion(
                                    local_solution_path_states_copy_[i], posEv->as<ob::State>()))
                            {
                                // ROS_INFO("%s:\n\tadding last position\n",
                                // ros::this_node::getName().c_str());
                                ob::ScopedState<> posEv(simple_setup_local_->getStateSpace());

                                posEv[0] = double(local_solution_path_states_copy_[i]
                                                      ->as<ob::SE2StateSpace::StateType>()
                                                      ->getX() +
                                                  (counter - 1) * robot_base_radius * std::cos(angle)); // x
                                posEv[1] = double(local_solution_path_states_copy_[i]
                                                      ->as<ob::SE2StateSpace::StateType>()
                                                      ->getY() +
                                                  (counter - 1) * robot_base_radius * std::sin(angle));

                                geometry_msgs::PoseStamped p;
                                p.pose.position.x = posEv[0];
                                p.pose.position.y = posEv[1];

                                if (goal_available_)
                                {
                                    ros::Time t;
                                    std::string err = "";
                                    tf::StampedTransform tf_map_to_fixed;
                                    tf_listener_.getLatestCommonTime("map", "odom", t, &err);
                                    tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);

                                    tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

                                    tf2::Quaternion myQuaternion;

                                    myQuaternion.setRPY(useless_roll, useless_pitch, goal_map_frame_[2] - yaw);

                                    myQuaternion = myQuaternion.normalize();

                                    p.pose.orientation.x = myQuaternion.getX();
                                    p.pose.orientation.y = myQuaternion.getY();
                                    p.pose.orientation.z = myQuaternion.getZ();
                                    p.pose.orientation.w = myQuaternion.getW();
                                }

                                lastNode = true;

                                path_visualize.append(posEv->as<ob::SE2StateSpace::StateType>());

                                solution_path_for_control.poses.push_back(p);
                            }
                            counter += 1;
                        }
                    }
                }
                // ROS_INFO("%s:\n\tpartial path sent\n", ros::this_node::getName().c_str());
                // ROS_INFO_STREAM("partial path: " << solution_path_for_control);
                visualizeRRTLocal(path_visualize);
                solution_path_control_pub_.publish(solution_path_for_control);
                // ros::spinOnce();
                //        if (mapping_offline_)
                //            goal_available_ = false;
            }
        }
    }
}

//! Resulting path visualization.
/*!
 * Visualize resulting path.
 */
void OnlinePlannFramework::visualizeRRT(og::PathGeometric &geopath)
{
    // %Tag(MARKER_INIT)%
    tf::Quaternion orien_quat;
    visualization_msgs::Marker visual_rrt, visual_result_path;
    visual_result_path.header.frame_id = visual_rrt.header.frame_id = world_frame_;
    visual_result_path.header.stamp = visual_rrt.header.stamp = ros::Time::now();
    visual_rrt.ns = "online_planner_rrt";
    visual_result_path.ns = "online_planner_result_path";
    visual_result_path.action = visual_rrt.action = visualization_msgs::Marker::ADD;

    visual_result_path.pose.orientation.w = visual_rrt.pose.orientation.w = 1.0;
    // %EndTag(MARKER_INIT)%

    // %Tag(ID)%
    visual_rrt.id = 0;
    visual_result_path.id = 1;
    // %EndTag(ID)%

    // %Tag(TYPE)%
    visual_rrt.type = visual_result_path.type = visualization_msgs::Marker::LINE_LIST;
    // %EndTag(TYPE)%

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    visual_rrt.scale.x = 0.03;
    visual_result_path.scale.x = 0.05;
    // %EndTag(SCALE)%

    // %Tag(COLOR)%
    // Points are green
    visual_result_path.color.g = 1.0;
    visual_result_path.color.a = 1.0;

    // Line strip is blue
    visual_rrt.color.b = 1.0;
    visual_rrt.color.a = 1.0;

    const ob::RealVectorStateSpace::StateType *state_r2;

    geometry_msgs::Point p;

    ob::PlannerData planner_data(simple_setup_global_->getSpaceInformation());
    simple_setup_global_->getPlannerData(planner_data);

    std::vector<unsigned int> edgeList;
    int num_parents;
    ROS_DEBUG("%s: number of states in the tree: %d", ros::this_node::getName().c_str(),
              planner_data.numVertices());

    if (visualize_tree_)
    {
        for (unsigned int i = 1; i < planner_data.numVertices(); ++i)
        {
            if (planner_data.getVertex(i).getState() && planner_data.getIncomingEdges(i, edgeList) > 0)
            {
                state_r2 = planner_data.getVertex(i).getState()->as<ob::RealVectorStateSpace::StateType>();
                p.x = state_r2->values[0];
                p.y = state_r2->values[1];
                p.z = 0.1;

                visual_rrt.points.push_back(p);

                state_r2 = planner_data.getVertex(edgeList[0]).getState()->as<ob::RealVectorStateSpace::StateType>();
                p.x = state_r2->values[0];
                p.y = state_r2->values[1];
                p.z = 0.1;

                visual_rrt.points.push_back(p);
            }
        }
        solution_path_rviz_pub_.publish(visual_rrt);
    }

    std::vector<ob::State *> states = geopath.getStates();
    for (uint32_t i = 0; i < geopath.getStateCount(); ++i)
    {
        // extract the component of the state and cast it to what we expect

        state_r2 = states[i]->as<ob::RealVectorStateSpace::StateType>();
        p.x = state_r2->values[0];
        p.y = state_r2->values[1];
        p.z = 0.1;

        if (i > 0)
        {
            visual_result_path.points.push_back(p);

            state_r2 = states[i - 1]->as<ob::RealVectorStateSpace::StateType>();
            p.x = state_r2->values[0];
            p.y = state_r2->values[1];
            p.z = 0.1;

            visual_result_path.points.push_back(p);
        }
    }
    solution_path_rviz_pub_.publish(visual_result_path);
}

void OnlinePlannFramework::visualizeRRTLocal(og::PathGeometric &geopath)
{
    // %Tag(MARKER_INIT)%
    tf::Quaternion orien_quat;
    visualization_msgs::Marker visual_rrt, visual_result_path;
    visual_result_path.header.frame_id = visual_rrt.header.frame_id = world_frame_;
    visual_result_path.header.stamp = visual_rrt.header.stamp = ros::Time::now();
    visual_rrt.ns = "online_planner_rrt_local";
    visual_result_path.ns = "online_planner_result_path_local";
    visual_result_path.action = visual_rrt.action = visualization_msgs::Marker::ADD;

    visual_result_path.pose.orientation.w = visual_rrt.pose.orientation.w = 1.0;
    // %EndTag(MARKER_INIT)%

    // %Tag(ID)%
    visual_rrt.id = 0;
    visual_result_path.id = 1;
    // %EndTag(ID)%

    // %Tag(TYPE)%
    visual_rrt.type = visual_result_path.type = visualization_msgs::Marker::LINE_LIST;
    // %EndTag(TYPE)%

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    visual_rrt.scale.x = 0.03;
    visual_result_path.scale.x = 0.075;
    // %EndTag(SCALE)%

    // %Tag(COLOR)%
    // Points are green
    visual_result_path.color.r = 1.0;
    visual_result_path.color.a = 1.0;

    // Line strip is blue
    visual_rrt.color.r = 0.0;
    visual_rrt.color.b = 0.0;
    visual_rrt.color.a = 1.0;

    const ob::SE2StateSpace::StateType *state_se2;

    geometry_msgs::Point p;

    ob::PlannerData planner_data(simple_setup_local_->getSpaceInformation());
    simple_setup_local_->getPlannerData(planner_data);

    std::vector<unsigned int> edgeList;
    int num_parents;
    ROS_DEBUG("%s: number of states in the tree: %d", ros::this_node::getName().c_str(),
              planner_data.numVertices());

    std_msgs::Int32 num_nodes;
    num_nodes.data = (int)planner_data.numVertices();

    num_nodes_pub_.publish(num_nodes);

    if (visualize_tree_)
    {
        for (unsigned int i = 1; i < planner_data.numVertices(); ++i)
        {
            if (planner_data.getVertex(i).getState() && planner_data.getIncomingEdges(i, edgeList) > 0)
            {

                state_se2 = planner_data.getVertex(i).getState()->as<ob::SE2StateSpace::StateType>();
                p.x = state_se2->getX();
                p.y = state_se2->getY();

                p.z = 0.1;

                visual_rrt.points.push_back(p);

                state_se2 = planner_data.getVertex(edgeList[0]).getState()->as<ob::SE2StateSpace::StateType>();
                p.x = state_se2->getX();
                p.y = state_se2->getY();

                p.z = 0.1;

                visual_rrt.points.push_back(p);
            }
        }
        solution_local_path_rviz_pub_.publish(visual_rrt);
    }

    std::vector<ob::State *> states = geopath.getStates();
    for (uint32_t i = 0; i < geopath.getStateCount(); ++i)
    {
        // extract the component of the state and cast it to what we expect

        const ob::SE2StateSpace::StateType *state_se2;
        state_se2 = states[i]->as<ob::SE2StateSpace::StateType>();
        p.x = state_se2->getX();
        p.y = state_se2->getY();

        p.z = 0.1;

        if (i > 0)
        {
            visual_result_path.points.push_back(p);

            const ob::SE2StateSpace::StateType *state_se2;
            state_se2 = states[i - 1]->as<ob::SE2StateSpace::StateType>();
            p.x = state_se2->getX();
            p.y = state_se2->getY();

            p.z = 0.1;

            visual_result_path.points.push_back(p);
        }
    }
    solution_local_path_rviz_pub_.publish(visual_result_path);
}

bool OnlinePlannFramework::validateGoalCandidate(const ob::ScopedState<> &goal_candidate)
{
    if (simple_setup_local_->getStateValidityChecker()->isValid(goal_candidate->as<ob::State>()))
    {
        return true;
    }
    ROS_WARN_STREAM("FIRST GOAL CANDIDATE INVALID!");
    return false;
}

ob::GoalStates *OnlinePlannFramework::findNewGoalCandidate(const ob::ScopedState<> &goal_candidate)
{

    oc::SpaceInformationPtr si_local = simple_setup_local_->getSpaceInformation();

    ob::GoalStates *goal_states(new ob::GoalStates(si_local));

    int i = 0;
    int j = 0;
    while (i < 50)
    {

        double r = local_goal_radius_ * std::sqrt(((double)rand() / (RAND_MAX)));
        double theta = (((double)rand() / (RAND_MAX))) * 2 * M_PI;
        double x = goal_candidate[0] + r * cos(theta);
        double y = goal_candidate[1] + r * sin(theta);

        ob::ScopedState<> new_goal_local(simple_setup_local_->getSpaceInformation()->getStateSpace());

        new_goal_local[0] = x;
        new_goal_local[1] = y;

        new_goal_local[2] = goal_candidate[2];

        if (simple_setup_local_->getStateValidityChecker()->isValid(new_goal_local->as<ob::State>()))
        {
            if (simple_setup_local_->getSpaceInformation()->checkMotion(goal_candidate->as<ob::State>(), new_goal_local->as<ob::State>()))
            {
                ROS_WARN_STREAM("APPROXIMATE VALID GOAL FOUND!");

                if (j < 20)
                {
                    goal_states->addState(new_goal_local);
                    j++;
                }
                else
                {
                    return goal_states;
                }
            }
        }
        i++;
    }
    ROS_WARN_STREAM("NO OPTIONAL GOAL FOUND!");

    if (j == 0)
    {
        goal_states->addState(goal_candidate);
    }

    return goal_states;
}

double OnlinePlannFramework::calculateAngle(double x1, double y1, double x2, double y2)
{
    double angle = atan2(y2 - y1, x2 - x1);
    return angle + (2.0 * M_PI * floor((M_PI - angle) / (2.0 * M_PI)));
}

//! Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "smf_move_base_planner");

    ROS_INFO("%s:\n\tonline planner (C++), using OMPL version %s\n", ros::this_node::getName().c_str(),
             OMPL_VERSION);
    // ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    //	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    //	   ros::console::notifyLoggerLevelsChanged();
    //	}

    OnlinePlannFramework online_planning_framework;
    online_planning_framework.planWithSimpleSetup();
    ros::spin();
    return 0;
}

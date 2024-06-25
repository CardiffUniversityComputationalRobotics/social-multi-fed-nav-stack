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
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

#include <planner/RRTstarMod.h>
#include <planner/InformedRRTstarMod.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <actionlib/server/simple_action_server.h>

// Planner
#include <new_state_sampler.h>
#include <informed_new_state_sampler.h>
#include <state_cost_objective.h>
#include <state_validity_checker_grid_map_R2.h>
#include <local_state_validity_checker_grid_map_R2.h>

// smf base controller
#include <smf_move_base_msgs/msg/path2_d.hpp>
#include <smf_move_base_msgs/action/goto2_d.hpp>

// pedsim msgs
#include <pedsim_msgs/msg/agent_states.hpp>
#include <pedsim_msgs/msg/agent_state.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// typedef actionlib::SimpleActionServer<smf_move_base_msgs::action::Goto2D> SmfBaseGoToActionServer;

//!  OnlinePlannFramework class.
/*!
 * Online Planning Framework.
 * Setup a sampling-based planner for online computation of collision-free paths.
 * C-Space: R2
 * Workspace is represented with Octomaps
 */
class OnlinePlannFramework : public rclcpp::Node
{
public:
    //! Constructor
    OnlinePlannFramework();
    //! Planner setup
    void planWithSimpleSetup();
    //! Periodic callback to solve the query.
    void planningTimerCallback();
    //! Callback for getting current vehicle odometry
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    //! Callback for getting the 2D navigation goal
    void queryGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr nav_goal_msg);
    //! Callback for getting the 2D navigation goal
    void goToActionCallback(const std::shared_ptr<smf_move_base_msgs::action::Goto2D::Goal> goto_req);
    //! Procedure to visualize the resulting path
    void visualizeRRT(og::PathGeometric &geopath);
    //! Procedure to visualize the resulting local path
    void visualizeRRTLocal(og::PathGeometric &geopath);
    //! Callback for getting the state of the Smf base controller.
    void controlActiveCallback(const std_msgs::msg::Bool::SharedPtr control_active_msg);
    //! check if goal candidate is valid
    bool validateGoalCandidate(const ob::ScopedState<> &goal_candidate);
    //! calculate angle between to points XY
    double calculateAngle(double x1, double y1, double x2, double y2);
    ob::GoalStates *findNewGoalCandidate(const ob::ScopedState<> &goal_candidate);

private:
    // ! SUBSCRIBERS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_active_sub_;

    // ! PUBLISHERS
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr solution_path_rviz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr solution_local_path_rviz_pub_;
    rclcpp::Publisher<smf_move_base_msgs::msg::Path2D>::SharedPtr solution_path_control_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr query_goal_pose_rviz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr query_goal_radius_rviz_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr num_nodes_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;

    // ! action server
    // SmfBaseGoToActionServer *goto_action_server_;
    // TODO: IMPLEMENTATION OF ACTION
    std::string goto_action_;
    smf_move_base_msgs::action::Goto2D::Feedback goto_action_feedback_;
    smf_move_base_msgs::action::Goto2D::Result goto_action_result_;

    // GRIDMAP SERVICE
    rclcpp::Client<GetGridMap>::SharedPtr grid_map_client_;
    std::string grid_map_service_;

    // ROS2 TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2::Transform last_robot_pose_;

    // OMPL, online planner
    og::SimpleSetupPtr simple_setup_global_, simple_setup_local_;
    double timer_period_, solving_time_, xy_goal_tolerance_, local_xy_goal_tolerance_, yaw_goal_tolerance_, robot_base_radius_;

    bool opport_collision_check_, reuse_last_best_solution_, local_reuse_last_best_solution_, motion_cost_interpolation_, odom_available_, goal_available_, dynamic_bounds_, visualize_tree_, control_active_, local_use_social_heatmap_;
    std::vector<double> planning_bounds_x_, planning_bounds_y_, start_state_, goal_map_frame_,
        goal_odom_frame_;
    double goal_radius_, local_goal_radius_, local_path_range_, global_time_percent_, turning_radius_;
    std::string planner_name_, local_planner_name_, optimization_objective_, local_optimization_objective_, odometry_topic_, query_goal_topic_, state_space_,
        solution_path_topic_, world_frame_, octomap_service_;
    std::vector<const ob::State *> solution_path_states_, local_solution_path_states_, past_local_solution_path_states_;

    nav_msgs::msg::Odometry::SharedPtr odom_data_;
    geometry_msgs::msg::Twist current_robot_velocity_;
};

OnlinePlannFramework::OnlinePlannFramework()
    : Node("online_planning_framework"), dynamic_bounds_(false), control_active_(false)
{
    //=======================================================================
    // TF LISTENER
    //=======================================================================
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //=======================================================================
    // Get parameters
    //=======================================================================
    planning_bounds_x_.resize(2);
    planning_bounds_y_.resize(2);
    start_state_.resize(2);
    goal_map_frame_.resize(3);
    goal_odom_frame_.resize(3);

    // ! DECLARE PARAMETERS
    this->declare_parameter("world_frame", rclcpp::ParameterValue(std::string("map")));
    this->declare_parameter("planning_bounds_x", rclcpp::ParameterValue(std::vector<double>{50.0, 50.0}));
    this->declare_parameter("planning_bounds_y", rclcpp::ParameterValue(std::vector<double>{50.0, 50.0}));
    this->declare_parameter("start_state", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0}));
    this->declare_parameter("goal_state", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
    this->declare_parameter("timer_period", rclcpp::ParameterValue(1.0));
    this->declare_parameter("solving_time", rclcpp::ParameterValue(1.0));
    this->declare_parameter("opport_collision_check", rclcpp::ParameterValue(false));
    this->declare_parameter("planner_name", rclcpp::ParameterValue(std::string("RRT")));
    this->declare_parameter("reuse_last_best_solution", rclcpp::ParameterValue(false));
    this->declare_parameter("local_reuse_last_best_solution", rclcpp::ParameterValue(false));
    this->declare_parameter("optimization_objective", rclcpp::ParameterValue(std::string("PathLength")));
    this->declare_parameter("motion_cost_interpolation", rclcpp::ParameterValue(false));
    this->declare_parameter("odometry_topic", rclcpp::ParameterValue(std::string("odom")));
    this->declare_parameter("query_goal_topic", rclcpp::ParameterValue(std::string("query_goal")));
    this->declare_parameter("goto_action", rclcpp::ParameterValue(std::string("go_action")));
    this->declare_parameter("solution_path_topic", rclcpp::ParameterValue(std::string("solution_path")));
    this->declare_parameter("dynamic_bounds", rclcpp::ParameterValue(false));
    this->declare_parameter("xy_goal_tolerance", rclcpp::ParameterValue(0.2));
    this->declare_parameter("yaw_goal_tolerance", rclcpp::ParameterValue(0.1));
    this->declare_parameter("visualize_tree", rclcpp::ParameterValue(false));
    this->declare_parameter("robot_base_radius", rclcpp::ParameterValue(0.0));
    this->declare_parameter("local_planner_name", rclcpp::ParameterValue(std::string("RRTstar")));
    this->declare_parameter("local_xy_goal_tolerance", rclcpp::ParameterValue(0.2));
    this->declare_parameter("local_optimization_objective", rclcpp::ParameterValue(std::string("PathLength")));
    this->declare_parameter("local_path_range", rclcpp::ParameterValue(0.0));
    this->declare_parameter("global_time_percent", rclcpp::ParameterValue(0.0));
    this->declare_parameter("turning_radius", rclcpp::ParameterValue(0.0));
    this->declare_parameter("state_space", rclcpp::ParameterValue(std::string("R2")));
    this->declare_parameter("grid_map_service", rclcpp::ParameterValue(std::string("grid_map_service")));
    this->declare_parameter("local_use_social_heatmap", rclcpp::ParameterValue(true));

    // ! GET PARAMETERS
    world_frame_ = this->get_parameter("world_frame").as_string();
    planning_bounds_x_ = this->get_parameter("planning_bounds_x").as_double_array();
    planning_bounds_y_ = this->get_parameter("planning_bounds_y").as_double_array();
    start_state_ = this->get_parameter("start_state").as_double_array();
    goal_map_frame_ = this->get_parameter("goal_state").as_double_array();
    timer_period_ = this->get_parameter("timer_period").as_double();
    solving_time_ = this->get_parameter("solving_time").as_double();
    opport_collision_check_ = this->get_parameter("opport_collision_check").as_bool();
    planner_name_ = this->get_parameter("planner_name").as_string();
    reuse_last_best_solution_ = this->get_parameter("reuse_last_best_solution").as_bool();
    local_reuse_last_best_solution_ = this->get_parameter("local_reuse_last_best_solution").as_bool();
    optimization_objective_ = this->get_parameter("optimization_objective").as_string();
    motion_cost_interpolation_ = this->get_parameter("motion_cost_interpolation").as_bool();
    odometry_topic_ = this->get_parameter("odometry_topic").as_string();
    query_goal_topic_ = this->get_parameter("query_goal_topic").as_string();
    goto_action_ = this->get_parameter("goto_action").as_string();
    solution_path_topic_ = this->get_parameter("solution_path_topic").as_string();
    dynamic_bounds_ = this->get_parameter("dynamic_bounds").as_bool();
    xy_goal_tolerance_ = this->get_parameter("xy_goal_tolerance").as_double();
    yaw_goal_tolerance_ = this->get_parameter("yaw_goal_tolerance").as_double();
    visualize_tree_ = this->get_parameter("visualize_tree").as_bool();
    robot_base_radius_ = this->get_parameter("robot_base_radius").as_double();
    local_planner_name_ = this->get_parameter("local_planner_name").as_string();
    local_xy_goal_tolerance_ = this->get_parameter("local_xy_goal_tolerance").as_double();
    local_optimization_objective_ = this->get_parameter("local_optimization_objective").as_string();
    local_path_range_ = this->get_parameter("local_path_range").as_double();
    global_time_percent_ = this->get_parameter("global_time_percent").as_double();
    turning_radius_ = this->get_parameter("turning_radius").as_double();
    state_space_ = this->get_parameter("state_space").as_string();
    grid_map_service_ = this->get_parameter("grid_map_service").as_string();
    local_use_social_heatmap_ = this->get_parameter("local_use_social_heatmap").as_bool();

    if (state_space_ == "dubins")
    {
        start_state_.resize(3);
    }

    goal_radius_ = xy_goal_tolerance_;
    local_goal_radius_ = local_xy_goal_tolerance_;
    goal_available_ = false;

    //=======================================================================
    // ! Subscribers
    //=======================================================================
    // Odometry data
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_, 1, std::bind(&OnlinePlannFramework::odomCallback, this, std::placeholders::_1));
    odom_available_ = false;

    // 2D Nav Goal
    nav_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(query_goal_topic_, 1, std::bind(&OnlinePlannFramework::queryGoalCallback, this, std::placeholders::_1));

    // Controller active flag
    // control_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(control_active_topic_, 1, std::bind(&OnlinePlannFramework::controlActiveCallback, this, std::placeholders::_1));

    //=======================================================================
    // ! Publishers
    //=======================================================================
    solution_path_rviz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("solution_path", 1);
    solution_local_path_rviz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("local_solution_path", 1);
    solution_path_control_pub_ = this->create_publisher<smf_move_base_msgs::msg::Path2D>(solution_path_topic_, 1);
    query_goal_pose_rviz_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("query_goal_pose_rviz", 1);
    query_goal_radius_rviz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("query_goal_radius_rviz", 1);
    num_nodes_pub_ = this->create_publisher<std_msgs::msg::Int32>("smf_num_nodes", 1);
    goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("goal_reached", 1);

    // ! SERVICE CLIENT WAIT
    grid_map_client_ = this->create_client<GetGridMap>(grid_map_service_);

    while (!grid_map_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    //=======================================================================
    // Action server
    //=======================================================================
    // goto_action_server_ = new SmfBaseGoToActionServer(
    //     this, goto_action_, std::bind(&OnlinePlannFramework::goToActionCallback, this, std::placeholders::_1), false);

    //=======================================================================
    // ! Waiting for odometry
    //=======================================================================
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok() && !odom_available_)
    {
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
        RCLCPP_WARN(this->get_logger(), "Waiting for vehicle's odometry");
    }
    RCLCPP_WARN(this->get_logger(), "Odometry received");

    // goto_action_server_->start();
}

//! Goto action callback.
/*!
 * Callback for getting the 2D navigation goal
 */
void OnlinePlannFramework::goToActionCallback(const std::shared_ptr<smf_move_base_msgs::action::Goto2D::Goal> goto_req)
{
    goal_map_frame_[0] = goto_req->goal.x;
    goal_map_frame_[1] = goto_req->goal.y;
    goal_map_frame_[2] = goto_req->goal.theta;

    double useless_pitch, useless_roll, yaw;

    //=======================================================================
    // Publish RViz Marker
    //=======================================================================
    geometry_msgs::msg::PoseStamped query_goal_msg;
    query_goal_msg.header.frame_id = "map";
    query_goal_msg.header.stamp = this->get_clock()->now();
    query_goal_msg.pose.position.x = goto_req->goal.x;
    query_goal_msg.pose.position.y = goto_req->goal.y;
    query_goal_msg.pose.position.z = 0.0;
    query_goal_msg.pose.orientation.x = 0.0;
    query_goal_msg.pose.orientation.y = 0.0;
    query_goal_msg.pose.orientation.z = sin(goto_req->goal.theta / 2.0);
    query_goal_msg.pose.orientation.w = cos(goto_req->goal.theta / 2.0);
    query_goal_pose_rviz_pub_->publish(query_goal_msg);

    visualization_msgs::msg::Marker radius_msg;
    radius_msg.header.frame_id = "map";
    radius_msg.header.stamp = this->get_clock()->now();
    radius_msg.ns = "goal_radius";
    radius_msg.action = visualization_msgs::msg::Marker::ADD;
    radius_msg.pose.orientation.w = 1.0;
    radius_msg.id = 0;
    radius_msg.type = visualization_msgs::msg::Marker::CYLINDER;
    radius_msg.scale.x = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.y = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.z = 0.02;
    radius_msg.color.r = 1.0;
    radius_msg.color.a = 0.5;
    radius_msg.pose.position.x = goto_req->goal.x;
    radius_msg.pose.position.y = goto_req->goal.y;
    radius_msg.pose.position.z = 0.0;
    query_goal_radius_rviz_pub_->publish(radius_msg);

    //=======================================================================
    // Transform from map to odom
    //=======================================================================
    geometry_msgs::msg::TransformStamped map_to_fixed;
    try
    {
        map_to_fixed = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
        tf2::Quaternion q(map_to_fixed.transform.rotation.x, map_to_fixed.transform.rotation.y,
                          map_to_fixed.transform.rotation.z, map_to_fixed.transform.rotation.w);
        tf2::Matrix3x3(q).getEulerYPR(yaw, useless_pitch, useless_roll);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform map to odom: %s", ex.what());
        return;
    }

    tf2::Transform tf_map_to_fixed(tf2::Quaternion(map_to_fixed.transform.rotation.x,
                                                   map_to_fixed.transform.rotation.y,
                                                   map_to_fixed.transform.rotation.z,
                                                   map_to_fixed.transform.rotation.w),
                                   tf2::Vector3(map_to_fixed.transform.translation.x,
                                                map_to_fixed.transform.translation.y,
                                                map_to_fixed.transform.translation.z));
    tf2::Vector3 goal_point_odom_frame(goal_map_frame_[0], goal_map_frame_[1], 0.0);
    goal_point_odom_frame = tf_map_to_fixed.inverse() * goal_point_odom_frame;
    goal_odom_frame_[0] = goal_point_odom_frame.x();
    goal_odom_frame_[1] = goal_point_odom_frame.y();
    goal_odom_frame_[2] = goal_map_frame_[2] - yaw;

    goal_radius_ = xy_goal_tolerance_;

    //=======================================================================
    // Clean and merge octomap
    //=======================================================================
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

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok() && (goal_available_ || control_active_))
        loop_rate.sleep();

    auto result = std::make_shared<smf_move_base_msgs::action::Goto2D::Result>();
    result->success = true;

    // goto_action_server_->succeeded(result);
    std_msgs::msg::Bool goal_reached;
    goal_reached.data = true;
    goal_reached_pub_->publish(goal_reached);
}

//! Odometry callback.
/*!
 * Callback for getting updated vehicle odometry
 */
void OnlinePlannFramework::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    if (!odom_available_)
        odom_available_ = true;

    geometry_msgs::msg::Pose predictedPose = odom_msg->pose.pose;

    predictedPose.position.x = odom_msg->pose.pose.position.x;

    predictedPose.position.y = odom_msg->pose.pose.position.y;

    tf2::fromMsg(predictedPose, last_robot_pose_);

    double useless_pitch,
        useless_roll, yaw;
    tf2::Matrix3x3(last_robot_pose_.getRotation()).getEulerYPR(yaw, useless_pitch, useless_roll);

    if ((goal_available_) &&
        sqrt(pow(goal_odom_frame_[0] - last_robot_pose_.getOrigin().getX(), 2.0) +
             pow(goal_odom_frame_[1] - last_robot_pose_.getOrigin().getY(), 2.0)) < (goal_radius_ + 0.2) &&
        abs(yaw - goal_odom_frame_[2]) < (yaw_goal_tolerance_ + 0.2))
    {
        goal_available_ = false;
        std_msgs::msg::Bool goal_reached;
        goal_reached.data = true;
        goal_reached_pub_->publish(goal_reached);
        RCLCPP_WARN(this->get_logger(), "Goal reached");
    }

    current_robot_velocity_ = odom_msg->twist.twist;
    odom_data_ = odom_msg;
}

//! Control active callback.
/*!
 * Callback for getting the state of the Smf base controller
 */
void OnlinePlannFramework::controlActiveCallback(const std_msgs::msg::Bool::SharedPtr control_active_msg)
{
    control_active_ = control_active_msg->data;
}

//! Navigation goal callback.
/*!
 * Callback for getting the 2D navigation goal
 */
void OnlinePlannFramework::queryGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr query_goal_msg)
{
    double useless_pitch, useless_roll, yaw;
    tf2::Quaternion q(query_goal_msg->pose.orientation.x, query_goal_msg->pose.orientation.y,
                      query_goal_msg->pose.orientation.z, query_goal_msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(useless_roll, useless_pitch, yaw);

    goal_map_frame_[0] = query_goal_msg->pose.position.x; // x
    goal_map_frame_[1] = query_goal_msg->pose.position.y; // y
    goal_map_frame_[2] = yaw;

    //=======================================================================
    // Transform from map to odom
    //=======================================================================
    geometry_msgs::msg::TransformStamped tf_map_to_fixed;
    try
    {
        tf_map_to_fixed = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform map to odom: %s", ex.what());
        return;
    }

    tf2::Transform tf2_map_to_fixed;
    tf2::fromMsg(tf_map_to_fixed.transform, tf2_map_to_fixed);
    tf2::Matrix3x3(tf2_map_to_fixed.getRotation()).getRPY(useless_roll, useless_pitch, yaw);

    tf2::Vector3 goal_point_odom_frame(goal_map_frame_[0], goal_map_frame_[1], 0.0);
    goal_point_odom_frame = tf2_map_to_fixed.inverse() * goal_point_odom_frame;
    goal_odom_frame_[0] = goal_point_odom_frame.x();
    goal_odom_frame_[1] = goal_point_odom_frame.y();
    goal_odom_frame_[2] = goal_map_frame_[2] - yaw;

    //=======================================================================
    // Clean and merge octomap
    //=======================================================================
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
    query_goal_pose_rviz_pub_->publish(*query_goal_msg);

    visualization_msgs::msg::Marker radius_msg;
    radius_msg.header.frame_id = "map";
    radius_msg.header.stamp = this->now();
    radius_msg.ns = "goal_radius";
    radius_msg.action = visualization_msgs::msg::Marker::ADD;
    radius_msg.pose.orientation.w = 1.0;
    radius_msg.id = 0;
    radius_msg.type = visualization_msgs::msg::Marker::CYLINDER;
    radius_msg.scale.x = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.y = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.z = 0.02;
    radius_msg.color.r = 1.0;
    radius_msg.color.a = 0.5;
    radius_msg.pose.position.x = goal_map_frame_[0];
    radius_msg.pose.position.y = goal_map_frame_[1];
    radius_msg.pose.position.z = 0.0;
    query_goal_radius_rviz_pub_->publish(radius_msg);
}

//!  Planner setup.
/*!
 * Setup a sampling-based planner using OMPL.
 */
void OnlinePlannFramework::planWithSimpleSetup()
{
    //=======================================================================
    // ! Instantiate the state space
    //=======================================================================
    ob::StateSpacePtr space = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));

    ob::StateSpacePtr local_space;

    if (state_space_.compare("dubins") == 0)
    {
        local_space = ob::StateSpacePtr(new ob::DubinsStateSpace(turning_radius_));
    }
    else
    {
        local_space = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));
    }

    //=======================================================================
    // ! Set the bounds for the state space
    //=======================================================================
    ob::RealVectorBounds bounds(2);

    bounds.setLow(0, planning_bounds_x_[0]);
    bounds.setHigh(0, planning_bounds_x_[1]);
    bounds.setLow(1, planning_bounds_y_[0]);
    bounds.setHigh(1, planning_bounds_y_[1]);

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    if (state_space_.compare("dubins") == 0)
    {
        local_space->as<ob::DubinsStateSpace>()->setBounds(bounds);
    }
    else
    {
        local_space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    }

    //=======================================================================
    // ! Define a simple setup class
    //=======================================================================
    // !defining simple setup for global planner
    simple_setup_global_ = og::SimpleSetupPtr(new og::SimpleSetup(space));
    ob::SpaceInformationPtr si_global = simple_setup_global_->getSpaceInformation();

    // !defining simple setup for local planner
    simple_setup_local_ = og::SimpleSetupPtr(new og::SimpleSetup(local_space));
    ob::SpaceInformationPtr si_local = simple_setup_local_->getSpaceInformation();

    // ! DUBINS MOTION VALIDATOR

    if (state_space_.compare("dubins") == 0)
    {
        ob::MotionValidatorPtr motion_validator;
        motion_validator = ob::MotionValidatorPtr(new ob::DubinsMotionValidator(si_local));
        si_local->setMotionValidator(motion_validator);
    }

    // ! ==================================

    //=======================================================================
    // ! Create a planner for the defined space
    //=======================================================================
    // ! GLOBAL PLANNER SETUP
    ob::PlannerPtr planner;
    if (planner_name_.compare("RRT") == 0)
        planner = ob::PlannerPtr(new og::RRT(si_global));
    if (planner_name_.compare("PRMstar") == 0)
        planner = ob::PlannerPtr(new og::PRMstar(si_global));
    else if (planner_name_.compare("RRTstar") == 0)
        planner = ob::PlannerPtr(new og::RRTstar(si_global));
    else if (planner_name_.compare("RRTstarMod") == 0)
        planner = ob::PlannerPtr(new og::RRTstarMod(si_global));
    else
        planner = ob::PlannerPtr(new og::RRTstar(si_global));

    // ! LOCAL PLANNER SETUP
    ob::PlannerPtr local_planner;
    if (local_planner_name_.compare("RRT") == 0)
        local_planner = ob::PlannerPtr(new og::RRT(si_local));
    if (local_planner_name_.compare("PRMstar") == 0)
        local_planner = ob::PlannerPtr(new og::PRMstar(si_local));
    else if (local_planner_name_.compare("RRTstar") == 0)
        local_planner = ob::PlannerPtr(new og::RRTstar(si_local));
    else if (local_planner_name_.compare("RRTstarMod") == 0)
        local_planner = ob::PlannerPtr(new og::RRTstarMod(si_local));
    else if (local_planner_name_.compare("InformedRRTstar") == 0)
        local_planner = ob::PlannerPtr(new og::InformedRRTstar(si_local));
    else if (local_planner_name_.compare("InformedRRTstarMod") == 0)
        local_planner = ob::PlannerPtr(new og::InformedRRTstarMod(si_local));
    else
        local_planner = ob::PlannerPtr(new og::RRTstar(si_local));

    //=======================================================================
    // ! Set the setup planner
    //=======================================================================
    simple_setup_global_->setPlanner(planner);

    simple_setup_local_->setPlanner(local_planner);

    //=======================================================================
    // ! Create a start and goal states
    //=======================================================================
    double useless_pitch, useless_roll, yaw;
    last_robot_pose_.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);
    start_state_[0] = double(last_robot_pose_.getOrigin().getX() + double(current_robot_velocity_.linear.x * (solving_time_ + 0.15))); // x
    start_state_[1] = double(last_robot_pose_.getOrigin().getY() + double(current_robot_velocity_.linear.y * (solving_time_ + 0.15))); // y

    if (state_space_.compare("dubins") == 0)
    {
        start_state_[2] = double(yaw);
    }

    // create a start state
    //! GLOBAL START STATE
    ob::ScopedState<> start(space);
    start[0] = double(start_state_[0]); // x
    start[1] = double(start_state_[1]); // y

    //! LOCAL START STATE
    ob::ScopedState<> local_start(local_space);
    local_start[0] = double(start_state_[0]); // x
    local_start[1] = double(start_state_[1]); // y
    if (state_space_.compare("dubins") == 0)
    {
        local_start[2] = double(start_state_[2]); // yaw
    }

    // create a goal state
    //! GLOBAL GOAL STATE
    ob::ScopedState<> goal(space);
    goal[0] = double(goal_map_frame_[0]); // x
    goal[1] = double(goal_map_frame_[1]); // y

    //! LOCAL GOAL STATE
    ob::ScopedState<> local_goal(local_space);
    local_goal[0] = double(goal_map_frame_[0]); // x
    local_goal[1] = double(goal_map_frame_[1]); // y
    if (state_space_.compare("dubins") == 0)
    {
        local_goal[2] = double(goal_map_frame_[2]); // yaw
    }

    //=======================================================================
    // ! Set the start and goal states
    //=======================================================================
    // ! GLOBAL GOAL SETUP
    simple_setup_global_->setStartState(start);
    simple_setup_global_->setGoalState(goal, goal_radius_);
    // simple_setup_->getStateSpace()->setValidSegmentCountFactor(5.0);

    // ! LOCAL GOAL SETUP
    simple_setup_local_->setStartState(local_start);
    simple_setup_local_->setGoalState(local_goal, local_goal_radius_);

    // =====================
    // ! GRID MAP REQUEST
    // =====================
    auto req = std::make_shared<GetGridMap::Request>();
    RCLCPP_INFO(this->get_logger(), "requesting the GridMap to %s", grid_map_service_.c_str());

    auto result = grid_map_client_->async_send_request(req);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_WARN(this->get_logger(), "Obtained GridMap");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading GridMap");
    }

    auto grid_map_msg = result.get()->map;

    //=======================================================================
    // ! Set state validity checking for this space
    //=======================================================================
    // !VALIDITY CHECKING FOR GLOBAL PLANNER
    ob::StateValidityCheckerPtr om_stat_val_check;
    om_stat_val_check = ob::StateValidityCheckerPtr(
        new GridMapStateValidityCheckerR2(simple_setup_global_->getSpaceInformation(), opport_collision_check_,
                                          planning_bounds_x_, planning_bounds_y_, grid_map_msg, robot_base_radius_));
    simple_setup_global_->setStateValidityChecker(om_stat_val_check);

    // !VALIDITY CHECKING FOR LOCAL PLANNER
    ob::StateValidityCheckerPtr local_om_stat_val_check;
    local_om_stat_val_check = ob::StateValidityCheckerPtr(
        new LocalGridMapStateValidityCheckerR2(simple_setup_local_->getSpaceInformation(), opport_collision_check_,
                                               planning_bounds_x_, planning_bounds_y_, grid_map_msg, robot_base_radius_, local_use_social_heatmap_));
    simple_setup_local_->setStateValidityChecker(local_om_stat_val_check);

    //=======================================================================
    // ! Set optimization objective
    //=======================================================================
    // !OPTIMIZATION OBJECTIVE FOR GLOBAL PLANNER
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
    // ! Perform setup steps for the planner
    //=======================================================================
    simple_setup_global_->setup();

    simple_setup_local_->setup();

    rclcpp::Rate loop_rate(1.0 / (timer_period_ - solving_time_)); // 10 hz
    // goal_available_ = true;
    while (rclcpp::ok())
    {
        if (goal_available_)
            RCLCPP_INFO(this->get_logger(), "goal available");
        OnlinePlannFramework::planningTimerCallback();
        rclcpp::spin_some(this->get_node_base_interface());
        // loop_rate.sleep();
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
        // ! Transform from map to odom
        //=======================================================================
        double useless_pitch, useless_roll, yaw;
        geometry_msgs::msg::TransformStamped tf_map_to_fixed;
        try
        {
            tf_map_to_fixed = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform map to odom: %s", ex.what());
            return;
        }

        tf2::Transform tf2_map_to_fixed;
        tf2::fromMsg(tf_map_to_fixed.transform, tf2_map_to_fixed);
        tf2::Matrix3x3(tf2_map_to_fixed.getRotation()).getRPY(useless_roll, useless_pitch, yaw);

        tf2::Vector3 goal_point_odom_frame(goal_map_frame_[0], goal_map_frame_[1], 0.0);
        goal_point_odom_frame = tf2_map_to_fixed.inverse() * goal_point_odom_frame;
        goal_odom_frame_[0] = goal_point_odom_frame.x();
        goal_odom_frame_[1] = goal_point_odom_frame.y();
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

            if (state_space_.compare("dubins") == 0)
            {
                simple_setup_local_->getStateSpace()->as<ob::DubinsStateSpace>()->setBounds(bounds);
            }
            else
            {
                simple_setup_local_->getStateSpace()->as<ob::RealVectorStateSpace>()->setBounds(bounds);
            }
        }

        //=======================================================================
        // ! Set new start state
        //=======================================================================

        ob::ScopedState<> start(simple_setup_global_->getSpaceInformation()->getStateSpace());
        ob::ScopedState<> goal(simple_setup_global_->getSpaceInformation()->getStateSpace());

        start[0] = double(last_robot_pose_.getOrigin().getX() + double(current_robot_velocity_.linear.x * (solving_time_ + 0.15))); // x
        start[1] = double(last_robot_pose_.getOrigin().getY() + double(current_robot_velocity_.linear.y * (solving_time_ + 0.15))); // y

        goal[0] = double(goal_odom_frame_[0]); // x
        goal[1] = double(goal_odom_frame_[1]); // y

        //======================================================================
        // Set the start and goal states
        //=======================================================================
        simple_setup_global_->clear();
        simple_setup_global_->clearStartStates();
        simple_setup_global_->setStartState(start);
        simple_setup_global_->setGoalState(goal, goal_radius_);
        simple_setup_global_->getStateSpace()->setValidSegmentCountFactor(5.0);

        //=======================================================================
        // ! Set a modified sampler
        //=======================================================================
        if (reuse_last_best_solution_)
            simple_setup_global_->getSpaceInformation()
                ->getStateSpace()
                ->setStateSamplerAllocator(
                    std::bind(newAllocStateSampler, std::placeholders::_1, simple_setup_global_->getPlanner(),
                              solution_path_states_));

        // ==================================
        // ! GRID MAP REQUEST
        // ==================================
        auto req = std::make_shared<GetGridMap::Request>();
        RCLCPP_INFO(this->get_logger(), "requesting the GridMap to %s", grid_map_service_.c_str());

        auto result = grid_map_client_->async_send_request(req);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_WARN(this->get_logger(), "Obtained GridMap");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading GridMap");
        }

        auto grid_map_msg = result.get()->map;

        //=======================================================================
        // ! Set state validity checking for this global space
        //=======================================================================
        ob::StateValidityCheckerPtr om_stat_val_check;
        om_stat_val_check = ob::StateValidityCheckerPtr(
            new GridMapStateValidityCheckerR2(simple_setup_global_->getSpaceInformation(), opport_collision_check_,
                                              planning_bounds_x_, planning_bounds_y_, grid_map_msg, robot_base_radius_));
        simple_setup_global_->setStateValidityChecker(om_stat_val_check);

        //=======================================================================
        // ! Set optimization objective
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

        last_robot_pose_.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

        simple_setup_local_->clearStartStates();
        ob::ScopedState<> local_start(simple_setup_local_->getSpaceInformation()->getStateSpace());
        ob::ScopedState<> local_goal(simple_setup_local_->getSpaceInformation()->getStateSpace());

        local_start[0] = double(last_robot_pose_.getOrigin().getX() + double(current_robot_velocity_.linear.x * (solving_time_ + 0.15))); // x
        local_start[1] = double(last_robot_pose_.getOrigin().getY() + double(current_robot_velocity_.linear.y * (solving_time_ + 0.15))); // y
        if (state_space_.compare("dubins") == 0)
        {
            local_start[2] = double(yaw); // yaw
        }

        simple_setup_local_->clear();
        simple_setup_local_->setStartState(local_start);
        simple_setup_local_->getStateSpace()->setValidSegmentCountFactor(15.0);

        //=======================================================================
        // ! Set state validity checking for this local space
        //=======================================================================
        ob::StateValidityCheckerPtr local_om_stat_val_check;
        local_om_stat_val_check = ob::StateValidityCheckerPtr(
            new LocalGridMapStateValidityCheckerR2(simple_setup_local_->getSpaceInformation(), opport_collision_check_,
                                                   planning_bounds_x_, planning_bounds_y_, grid_map_msg, robot_base_radius_, local_use_social_heatmap_));
        simple_setup_local_->setStateValidityChecker(local_om_stat_val_check);

        //=======================================================================
        // ! Attempt to solve the problem of global planner
        //=======================================================================
        ob::PlannerStatus solved = simple_setup_global_->solve(solving_time_ * (global_time_percent_ / 100));

        bool solution_found = false;

        if (solved && simple_setup_global_->haveExactSolutionPath())
        {
            solution_found = true;
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path

            og::PathGeometric path = simple_setup_global_->getSolutionPath();

            // generates varios little segments for the waypoints obtained from the planner
            path.interpolate(int(path.length() / 0.5));

            RCLCPP_INFO(this->get_logger(), "\n\tpath with cost %f has been found with simple_setup\n",
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
                    // solution_path_states_.reserve(path_states.size());
                    for (int i = path_states.size() - 1; i >= 0; i--)
                    {
                        ob::State *s = space->allocState();
                        space->copyState(s, path_states[i]);
                        solution_path_states_.push_back(s);
                    }
                }

                og::PathGeometric global_path_solution = og::PathGeometric(simple_setup_global_->getSpaceInformation());

                for (int i = 0; i < solution_path_states_.size(); i++)
                {
                    global_path_solution.append(solution_path_states_[i]);
                }

                visualizeRRT(global_path_solution);

                //=======================================================================
                // ! LOCAL PLANNER SOLVE
                //=======================================================================

                std::vector<const ob::State *> global_path_feedback;
                ob::StateSpacePtr local_space = simple_setup_local_->getStateSpace();

                int states_num_limit = solution_path_states_.size() - double(local_path_range_ / 0.5) - 4;

                if (states_num_limit < 0)
                {
                    states_num_limit = 0;
                }

                for (int i = solution_path_states_.size() - 1; i > states_num_limit; i--)
                {
                    double local_path_distance = std::sqrt(std::pow(start[0] - solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[0], 2) + std::pow(start[1] - solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[1], 2));

                    ob::State *s = local_space->allocState();

                    if (state_space_.compare("dubins") == 0)
                    {
                        s->as<ob::DubinsStateSpace::StateType>()->setX(solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[0]);
                        s->as<ob::DubinsStateSpace::StateType>()->setY(solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[1]);

                        double state_angle = calculateAngle(solution_path_states_[i - 1]->as<ob::RealVectorStateSpace::StateType>()->values[1],
                                                            solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[1],
                                                            solution_path_states_[i - 1]->as<ob::RealVectorStateSpace::StateType>()->values[0],
                                                            solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[0]);

                        s->as<ob::DubinsStateSpace::StateType>()->setYaw(state_angle);
                        local_goal[2] = double(state_angle);
                    }
                    else
                    {
                        local_space->copyState(s, solution_path_states_[i]);
                    }

                    global_path_feedback.push_back(s);

                    local_goal[0] = double(solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[0]); // x
                    local_goal[1] = double(solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[1]); // y

                    if (local_path_distance >= local_path_range_)
                    {
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
                // ! Set goal state for local planner
                //=======================================================================

                if (validateGoalCandidate(local_goal))
                {

                    double waypoints_diff_x;
                    double waypoints_diff_y;

                    if (state_space_.compare("dubins") == 0)
                    {
                        waypoints_diff_x = abs(global_path_feedback[global_path_feedback.size() - 1]->as<ob::DubinsStateSpace::StateType>()->getX() - goal[0]);
                        waypoints_diff_y = abs(global_path_feedback[global_path_feedback.size() - 1]->as<ob::DubinsStateSpace::StateType>()->getY() - goal[1]);
                    }
                    else
                    {
                        waypoints_diff_x = abs(global_path_feedback[global_path_feedback.size() - 1]->as<ob::RealVectorStateSpace::StateType>()->values[0] - goal[0]);
                        waypoints_diff_y = abs(global_path_feedback[global_path_feedback.size() - 1]->as<ob::RealVectorStateSpace::StateType>()->values[1] - goal[1]);
                    }

                    if (waypoints_diff_x < 3 && waypoints_diff_y < 3)
                    {
                        local_goal[0] = double(goal[0]); // x
                        local_goal[1] = double(goal[1]); // y

                        if (state_space_.compare("dubins") == 0)
                        {
                            local_goal[2] = double(goal[2]); // yaw
                        }

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
                // ! Set a modified local sampler
                //=======================================================================

                if (reuse_last_best_solution_)
                {

                    //=======================================================================
                    // Set optimization objective
                    //=======================================================================
                    if (local_optimization_objective_.compare("PathLength") == 0) // path length Objective
                        simple_setup_local_->getProblemDefinition()->setOptimizationObjective(
                            getPathLengthObjective(simple_setup_local_->getSpaceInformation()));
                    else if (local_optimization_objective_.compare("SocialComfort") == 0) // Social Comfort
                        simple_setup_local_->getProblemDefinition()->setOptimizationObjective(
                            getSocialComfortObjective(simple_setup_local_->getSpaceInformation(), motion_cost_interpolation_, local_space, simple_setup_global_->getPlanner(), global_path_feedback));
                    else if (local_optimization_objective_.compare("SocialHeatmap") == 0) // Social Costmap
                        simple_setup_local_->getProblemDefinition()->setOptimizationObjective(
                            getSocialHeatmapObjective(simple_setup_local_->getSpaceInformation(), motion_cost_interpolation_));
                    else
                        simple_setup_local_->getProblemDefinition()->setOptimizationObjective(
                            getPathLengthObjective(simple_setup_local_->getSpaceInformation()));
                }

                //=======================================================================
                // ! Attempt to solve the problem of the local planner
                //=======================================================================
                ob::PlannerStatus solved_local = simple_setup_local_->solve(solving_time_ * (1 - (global_time_percent_ / 100)));

                if (solved_local && simple_setup_local_->haveExactSolutionPath())
                {
                    solution_found = true;
                    // get the goal representation from the problem definition (not the same as the goal state)
                    // and inquire about the found path

                    og::PathGeometric path_local = simple_setup_local_->getSolutionPath();

                    // generates varios little segments for the waypoints obtained from the planner
                    path_local.interpolate(int(path_local.length() / 0.2));

                    // path_planning_msgs::PathConstSpeed solution_path;
                    RCLCPP_INFO(this->get_logger(), "\n\tlocal path with cost %f has been found with simple_setup\n",
                                path_local.cost(simple_setup_local_->getProblemDefinition()->getOptimizationObjective()).value());

                    std::vector<ob::State *> local_path_states;
                    local_path_states = path_local.getStates();

                    double distance_to_goal;
                    if (state_space_.compare("dubins") == 0)
                    {
                        distance_to_goal =
                            sqrt(pow(local_goal[0] - local_path_states[local_path_states.size() - 1]
                                                         ->as<ob::DubinsStateSpace::StateType>()
                                                         ->getX(),
                                     2.0) +
                                 pow(local_goal[1] - local_path_states[local_path_states.size() - 1]
                                                         ->as<ob::DubinsStateSpace::StateType>()
                                                         ->getY(),
                                     2.0));
                    }
                    else
                    {
                        distance_to_goal =
                            sqrt(pow(goal_odom_frame_[0] - path_states[path_states.size() - 1]
                                                               ->as<ob::RealVectorStateSpace::StateType>()
                                                               ->values[0],
                                     2.0) +
                                 pow(goal_odom_frame_[1] - path_states[path_states.size() - 1]
                                                               ->as<ob::RealVectorStateSpace::StateType>()
                                                               ->values[1],
                                     2.0));
                    }

                    std::vector<ob::State *> controller_local_path_states;

                    if (simple_setup_local_->haveExactSolutionPath() || distance_to_goal <= local_goal_radius_)
                    {
                        // ======================================================================
                        ob::StateSpacePtr space = simple_setup_local_->getStateSpace();

                        ob::ScopedState<> current_robot_state(simple_setup_local_->getSpaceInformation()->getStateSpace());
                        current_robot_state[0] = odom_data_->pose.pose.position.x;
                        current_robot_state[1] = odom_data_->pose.pose.position.y;

                        // !NEAREST POINT FROM PAST LOCAL PATH TO APPEND IN FINAL SOLUTION
                        if (past_local_solution_path_states_.size() > 0)
                        {
                            double init_x_distance = 10000;
                            double init_y_distance = 10000;
                            int less_distance_index = 0;
                            for (int i = 0; i < past_local_solution_path_states_.size(); i++)
                            {
                                double current_distance_x;
                                double current_distance_y;

                                if (state_space_.compare("dubins") == 0)
                                {
                                    current_distance_x = abs(odom_data_->pose.pose.position.x - past_local_solution_path_states_[i]->as<ob::DubinsStateSpace::StateType>()->getX());
                                    current_distance_y = abs(odom_data_->pose.pose.position.y - past_local_solution_path_states_[i]->as<ob::DubinsStateSpace::StateType>()->getY());
                                }
                                else
                                {
                                    current_distance_x = abs(odom_data_->pose.pose.position.x - past_local_solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[0]);
                                    current_distance_y = abs(odom_data_->pose.pose.position.y - past_local_solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[1]);
                                }

                                if (current_distance_x < init_x_distance || current_distance_y < init_y_distance)
                                {

                                    if (simple_setup_local_->getSpaceInformation()->checkMotion(past_local_solution_path_states_[i], current_robot_state->as<ob::State>()))
                                    {
                                        init_x_distance = current_distance_x;
                                        init_y_distance = current_distance_y;
                                        less_distance_index = i;
                                    }
                                }
                            }

                            if (less_distance_index != 0)
                            {

                                int max_limit_index = less_distance_index + 6;

                                if (max_limit_index > past_local_solution_path_states_.size())
                                {
                                    max_limit_index = past_local_solution_path_states_.size();
                                }

                                for (int i = less_distance_index; i < max_limit_index; i++)
                                {
                                    ob::State *s = space->allocState();
                                    space->copyState(s, past_local_solution_path_states_[i]);
                                    controller_local_path_states.push_back(s);
                                }
                            }
                        }

                        std::reverse(controller_local_path_states.begin(), controller_local_path_states.end());

                        ob::State *s = space->allocState();
                        space->copyState(s, current_robot_state->as<ob::State>());
                        controller_local_path_states.push_back(s);

                        // ====================================

                        // !NEAREST POINT FROM CURRENT LOCAL PATH SOLUTION
                        // ! TRIM LOCAL FOUND IF DUBINS NOT USED

                        double init_x_distance = 10000;
                        double init_y_distance = 10000;
                        int less_distance_index = 0;
                        for (int i = (local_path_states.size() - 1); i > -1; i--)
                        {
                            double current_distance_x;
                            double current_distance_y;

                            if (state_space_.compare("dubins") == 0)
                            {
                                current_distance_x = abs(odom_data_->pose.pose.position.x - local_path_states[i]->as<ob::DubinsStateSpace::StateType>()->getX());
                                current_distance_y = abs(odom_data_->pose.pose.position.y - local_path_states[i]->as<ob::DubinsStateSpace::StateType>()->getY());
                            }
                            else
                            {
                                current_distance_x = abs(odom_data_->pose.pose.position.x - local_path_states[i]->as<ob::RealVectorStateSpace::StateType>()->values[0]);
                                current_distance_y = abs(odom_data_->pose.pose.position.y - local_path_states[i]->as<ob::RealVectorStateSpace::StateType>()->values[1]);
                            }

                            if (current_distance_x < init_x_distance || current_distance_y < init_y_distance)
                            {
                                if (simple_setup_local_->getSpaceInformation()->checkMotion(local_path_states[i], current_robot_state->as<ob::State>()))
                                {
                                    init_x_distance = current_distance_x;
                                    init_y_distance = current_distance_y;
                                    less_distance_index = i;

                                    if (current_distance_x < 0.4 && current_distance_y < 0.4)
                                    {
                                        break;
                                    }
                                }
                            }
                        }

                        for (int i = less_distance_index; i < local_path_states.size(); i++)
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
                            for (int i = local_path_states.size() - 1; i >= 0; i--)
                            {
                                ob::State *s = space->allocState();
                                space->copyState(s, local_path_states[i]);
                                local_solution_path_states_.push_back(s);
                                past_local_solution_path_states_.push_back(s);
                            }
                        }
                    }

                    // =======================
                    // !end of local planner solve
                    // =======================

                    //=======================================================================
                    // ! Controller
                    //=======================================================================
                    if (controller_local_path_states.size() > 0)
                    {
                        smf_move_base_msgs::msg::Path2D solution_path_for_control;
                        for (unsigned int i = 0; i < controller_local_path_states.size(); i++)
                        {
                            geometry_msgs::msg::Pose2D p;

                            if (state_space_.compare("dubins") == 0)
                            {
                                p.x = controller_local_path_states[i]->as<ob::DubinsStateSpace::StateType>()->getX();
                                p.y = controller_local_path_states[i]->as<ob::DubinsStateSpace::StateType>()->getY();
                            }
                            else
                            {
                                p.x = controller_local_path_states[i]->as<ob::RealVectorStateSpace::StateType>()->values[0];
                                p.y = controller_local_path_states[i]->as<ob::RealVectorStateSpace::StateType>()->values[1];
                            }

                            if (i == (controller_local_path_states.size() - 1))
                            {
                                if (goal_available_)
                                {

                                    p.theta = goal_map_frame_[2];
                                }
                            }
                            solution_path_for_control.waypoints.push_back(p);
                        }
                        // ROS_INFO_STREAM("complete path: " << solution_path_for_control);
                        solution_path_control_pub_->publish(solution_path_for_control);
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

            solution_found = false;
        }

        // =======================
        // ! IF PATH IS NOT FOUND
        // =======================
        if (!solution_found)
        {
            RCLCPP_WARN(this->get_logger(), "\n\tpath has not been found\n");

            simple_setup_local_->clear();

            ob::StateValidityCheckerPtr local_om_stat_val_check;
            local_om_stat_val_check = ob::StateValidityCheckerPtr(
                new LocalGridMapStateValidityCheckerR2(simple_setup_local_->getSpaceInformation(), opport_collision_check_,
                                                       planning_bounds_x_, planning_bounds_y_, grid_map_msg, robot_base_radius_, local_use_social_heatmap_));
            simple_setup_local_->setStateValidityChecker(local_om_stat_val_check);

            if (past_local_solution_path_states_.size() > 0)
            {
                std::vector<const ob::State *> local_solution_path_states_copy_;

                ob::StateSpacePtr space = simple_setup_local_->getStateSpace();

                ob::ScopedState<> current_robot_state(simple_setup_local_->getSpaceInformation()->getStateSpace());
                current_robot_state[0] = odom_data_->pose.pose.position.x;
                current_robot_state[1] = odom_data_->pose.pose.position.y;

                double init_x_distance = 10000;
                double init_y_distance = 10000;
                int less_distance_index = 0;
                for (int i = 0; i < past_local_solution_path_states_.size(); i++)
                {

                    double current_distance_x;
                    double current_distance_y;

                    if (state_space_.compare("dubins") == 0)
                    {
                        current_distance_x = abs(odom_data_->pose.pose.position.x - past_local_solution_path_states_[i]->as<ob::DubinsStateSpace::StateType>()->getX());
                        current_distance_y = abs(odom_data_->pose.pose.position.y - past_local_solution_path_states_[i]->as<ob::DubinsStateSpace::StateType>()->getY());
                    }
                    else
                    {
                        current_distance_x = abs(odom_data_->pose.pose.position.x - past_local_solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[0]);
                        current_distance_y = abs(odom_data_->pose.pose.position.y - past_local_solution_path_states_[i]->as<ob::RealVectorStateSpace::StateType>()->values[1]);
                    }

                    if (current_distance_x < init_x_distance || current_distance_y < init_y_distance)
                    {
                        init_x_distance = current_distance_x;
                        init_y_distance = current_distance_y;
                        less_distance_index = i;
                    }
                }

                less_distance_index += 1;
                if (less_distance_index > past_local_solution_path_states_.size())
                {
                    less_distance_index = past_local_solution_path_states_.size();
                }

                for (int i = 0; i < less_distance_index; i++)
                {
                    ob::State *s = space->allocState();
                    space->copyState(s, past_local_solution_path_states_[i]);
                    local_solution_path_states_copy_.push_back(s);
                }

                std::reverse(local_solution_path_states_copy_.begin(), local_solution_path_states_copy_.end());
                RCLCPP_WARN(this->get_logger(), "sending partial last possible path\n");
                smf_move_base_msgs::msg::Path2D solution_path_for_control;
                og::PathGeometric path_visualize = og::PathGeometric(simple_setup_local_->getSpaceInformation());

                // adding first waypoint
                if (simple_setup_local_->getStateValidityChecker()->isValid(local_solution_path_states_copy_[0]))
                {
                    geometry_msgs::msg::Pose2D p;
                    if (state_space_.compare("dubins") == 0)
                    {
                        p.x = local_solution_path_states_copy_[0]->as<ob::DubinsStateSpace::StateType>()->getX();
                        p.y = local_solution_path_states_copy_[0]->as<ob::DubinsStateSpace::StateType>()->getY();
                    }
                    else
                    {
                        p.x = local_solution_path_states_copy_[0]->as<ob::RealVectorStateSpace::StateType>()->values[0];
                        p.y = local_solution_path_states_copy_[0]->as<ob::RealVectorStateSpace::StateType>()->values[1];
                    }

                    if (0 == (local_solution_path_states_copy_.size() - 1))
                    {
                        if (goal_available_)
                        {

                            p.theta = goal_map_frame_[2];
                        }
                    }
                    solution_path_for_control.waypoints.push_back(p);
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

                        geometry_msgs::msg::Pose2D p;

                        if (state_space_.compare("dubins") == 0)
                        {
                            p.x = local_solution_path_states_copy_[i + 1]
                                      ->as<ob::DubinsStateSpace::StateType>()
                                      ->getX();
                            p.y = local_solution_path_states_copy_[i + 1]
                                      ->as<ob::DubinsStateSpace::StateType>()
                                      ->getY();
                        }
                        else
                        {
                            p.x = local_solution_path_states_copy_[i + 1]
                                      ->as<ob::RealVectorStateSpace::StateType>()
                                      ->values[0];
                            p.y = local_solution_path_states_copy_[i + 1]
                                      ->as<ob::RealVectorStateSpace::StateType>()
                                      ->values[1];
                        }

                        if (i == (local_solution_path_states_copy_.size() - 1))
                        {
                            if (goal_available_)
                            {

                                p.theta = goal_map_frame_[2];
                            }
                        }
                        solution_path_for_control.waypoints.push_back(p);
                        path_visualize.append(local_solution_path_states_copy_[i + 1]);
                    }
                    else
                    {
                        double angle;

                        if (state_space_.compare("dubins") == 0)
                        {
                            angle = atan2(local_solution_path_states_copy_[i + 1]
                                                  ->as<ob::DubinsStateSpace::StateType>()
                                                  ->getY() -
                                              local_solution_path_states_copy_[i]
                                                  ->as<ob::DubinsStateSpace::StateType>()
                                                  ->getY(),
                                          local_solution_path_states_copy_[i + 1]
                                                  ->as<ob::DubinsStateSpace::StateType>()
                                                  ->getX() -
                                              local_solution_path_states_copy_[i]
                                                  ->as<ob::DubinsStateSpace::StateType>()
                                                  ->getX());
                        }
                        else
                        {
                            angle = atan2(local_solution_path_states_copy_[i + 1]
                                                  ->as<ob::RealVectorStateSpace::StateType>()
                                                  ->values[1] -
                                              local_solution_path_states_copy_[i]
                                                  ->as<ob::RealVectorStateSpace::StateType>()
                                                  ->values[1],
                                          local_solution_path_states_copy_[i + 1]
                                                  ->as<ob::RealVectorStateSpace::StateType>()
                                                  ->values[0] -
                                              local_solution_path_states_copy_[i]
                                                  ->as<ob::RealVectorStateSpace::StateType>()
                                                  ->values[0]);
                        }

                        int counter = 1;
                        while (!lastNode)
                        {
                            ob::ScopedState<> posEv(simple_setup_local_->getStateSpace());

                            if (state_space_.compare("dubins") == 0)
                            {
                                posEv[0] = double(local_solution_path_states_copy_[i]
                                                      ->as<ob::DubinsStateSpace::StateType>()
                                                      ->getX() +
                                                  counter * robot_base_radius_ * std::cos(angle)); // x
                                posEv[1] = double(local_solution_path_states_copy_[i]
                                                      ->as<ob::DubinsStateSpace::StateType>()
                                                      ->getY() +
                                                  counter * robot_base_radius_ * std::sin(angle)); // y
                            }
                            else
                            {
                                posEv[0] = double(local_solution_path_states_copy_[i]
                                                      ->as<ob::RealVectorStateSpace::StateType>()
                                                      ->values[0] +
                                                  counter * robot_base_radius_ * std::cos(angle)); // x
                                posEv[1] = double(local_solution_path_states_copy_[i]
                                                      ->as<ob::RealVectorStateSpace::StateType>()
                                                      ->values[1] +
                                                  counter * robot_base_radius_ * std::sin(angle)); // y
                            }

                            if (!simple_setup_local_->getSpaceInformation()->checkMotion(
                                    local_solution_path_states_copy_[i], posEv->as<ob::State>()))
                            {
                                ob::ScopedState<> posEv(simple_setup_local_->getStateSpace());

                                if (state_space_.compare("dubins") == 0)
                                {
                                    posEv[0] = double(local_solution_path_states_copy_[i]
                                                          ->as<ob::DubinsStateSpace::StateType>()
                                                          ->getX() +
                                                      (counter - 1) * robot_base_radius_ * std::cos(angle)); // x
                                    posEv[1] = double(local_solution_path_states_copy_[i]
                                                          ->as<ob::DubinsStateSpace::StateType>()
                                                          ->getY() +
                                                      (counter - 1) * robot_base_radius_ * std::sin(angle));
                                }
                                else
                                {
                                    posEv[0] = double(local_solution_path_states_copy_[i]
                                                          ->as<ob::RealVectorStateSpace::StateType>()
                                                          ->values[0] +
                                                      (counter - 1) * robot_base_radius_ * std::cos(angle)); // x
                                    posEv[1] = double(local_solution_path_states_copy_[i]
                                                          ->as<ob::RealVectorStateSpace::StateType>()
                                                          ->values[1] +
                                                      (counter - 1) * robot_base_radius_ * std::sin(angle));
                                }

                                geometry_msgs::msg::Pose2D p;
                                p.x = posEv[0];
                                p.y = posEv[1];

                                if (goal_available_)
                                {

                                    p.theta = goal_map_frame_[2];
                                }

                                lastNode = true;

                                if (state_space_.compare("dubins") == 0)
                                {
                                    path_visualize.append(posEv->as<ob::DubinsStateSpace::StateType>());
                                }
                                else
                                {
                                    path_visualize.append(posEv->as<ob::DubinsStateSpace::StateType>());
                                }

                                solution_path_for_control.waypoints.push_back(p);
                            }
                            counter += 1;
                        }
                    }
                }
                visualizeRRTLocal(path_visualize);
                solution_path_control_pub_->publish(solution_path_for_control);
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
    tf2::Quaternion orien_quat;
    visualization_msgs::msg::Marker visual_rrt, visual_result_path;
    visual_result_path.header.frame_id = visual_rrt.header.frame_id = world_frame_;
    visual_result_path.header.stamp = visual_rrt.header.stamp = this->now();
    visual_rrt.ns = "online_planner_rrt";
    visual_result_path.ns = "online_planner_result_path";
    visual_result_path.action = visual_rrt.action = visualization_msgs::msg::Marker::ADD;

    visual_result_path.pose.orientation.w = visual_rrt.pose.orientation.w = 1.0;
    // %EndTag(MARKER_INIT)%

    // %Tag(ID)%
    visual_rrt.id = 0;
    visual_result_path.id = 1;
    // %EndTag(ID)%

    // %Tag(TYPE)%
    visual_rrt.type = visual_result_path.type = visualization_msgs::msg::Marker::LINE_LIST;
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

    geometry_msgs::msg::Point p;

    ob::PlannerData planner_data(simple_setup_global_->getSpaceInformation());
    simple_setup_global_->getPlannerData(planner_data);

    std::vector<unsigned int> edgeList;
    RCLCPP_DEBUG(this->get_logger(), "number of states in the tree: %d",
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
        solution_path_rviz_pub_->publish(visual_rrt);
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
    solution_path_rviz_pub_->publish(visual_result_path);
}

void OnlinePlannFramework::visualizeRRTLocal(og::PathGeometric &geopath)
{
    // %Tag(MARKER_INIT)%
    tf2::Quaternion orien_quat;
    visualization_msgs::msg::Marker visual_rrt, visual_result_path;
    visual_result_path.header.frame_id = visual_rrt.header.frame_id = world_frame_;
    visual_result_path.header.stamp = visual_rrt.header.stamp = this->now();
    visual_rrt.ns = "online_planner_rrt_local";
    visual_result_path.ns = "online_planner_result_path_local";
    visual_result_path.action = visual_rrt.action = visualization_msgs::msg::Marker::ADD;

    visual_result_path.pose.orientation.w = visual_rrt.pose.orientation.w = 1.0;
    // %EndTag(MARKER_INIT)%

    // %Tag(ID)%
    visual_rrt.id = 0;
    visual_result_path.id = 1;
    // %EndTag(ID)%

    // %Tag(TYPE)%
    visual_rrt.type = visual_result_path.type = visualization_msgs::msg::Marker::LINE_LIST;
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

    geometry_msgs::msg::Point p;

    ob::PlannerData planner_data(simple_setup_local_->getSpaceInformation());
    simple_setup_local_->getPlannerData(planner_data);

    std::vector<unsigned int> edgeList;
    RCLCPP_DEBUG(this->get_logger(), "number of states in the tree: %d",
                 planner_data.numVertices());

    std_msgs::msg::Int32 num_nodes;
    num_nodes.data = static_cast<int>(planner_data.numVertices());

    num_nodes_pub_->publish(num_nodes);

    if (visualize_tree_)
    {
        for (unsigned int i = 1; i < planner_data.numVertices(); ++i)
        {
            if (planner_data.getVertex(i).getState() && planner_data.getIncomingEdges(i, edgeList) > 0)
            {
                if (state_space_.compare("dubins") == 0)
                {
                    const ob::DubinsStateSpace::StateType *state_r2;
                    state_r2 = planner_data.getVertex(i).getState()->as<ob::DubinsStateSpace::StateType>();
                    p.x = state_r2->getX();
                    p.y = state_r2->getY();
                }
                else
                {
                    const ob::RealVectorStateSpace::StateType *state_r2;
                    state_r2 = planner_data.getVertex(i).getState()->as<ob::RealVectorStateSpace::StateType>();
                    p.x = state_r2->values[0];
                    p.y = state_r2->values[1];
                }

                p.z = 0.1;

                visual_rrt.points.push_back(p);

                if (state_space_.compare("dubins") == 0)
                {
                    const ob::DubinsStateSpace::StateType *state_r2;
                    state_r2 = planner_data.getVertex(edgeList[0]).getState()->as<ob::DubinsStateSpace::StateType>();
                    p.x = state_r2->getX();
                    p.y = state_r2->getY();
                }
                else
                {
                    const ob::RealVectorStateSpace::StateType *state_r2;
                    state_r2 = planner_data.getVertex(edgeList[0]).getState()->as<ob::RealVectorStateSpace::StateType>();
                    p.x = state_r2->values[0];
                    p.y = state_r2->values[1];
                }

                p.z = 0.1;

                visual_rrt.points.push_back(p);
            }
        }
        solution_local_path_rviz_pub_->publish(visual_rrt);
    }

    std::vector<ob::State *> states = geopath.getStates();
    for (uint32_t i = 0; i < geopath.getStateCount(); ++i)
    {
        // extract the component of the state and cast it to what we expect

        if (state_space_.compare("dubins") == 0)
        {
            const ob::DubinsStateSpace::StateType *state_r2;
            state_r2 = states[i]->as<ob::DubinsStateSpace::StateType>();
            p.x = state_r2->getX();
            p.y = state_r2->getY();
        }
        else
        {
            const ob::RealVectorStateSpace::StateType *state_r2;
            state_r2 = states[i]->as<ob::RealVectorStateSpace::StateType>();
            p.x = state_r2->values[0];
            p.y = state_r2->values[1];
        }

        p.z = 0.1;

        if (i > 0)
        {
            visual_result_path.points.push_back(p);

            if (state_space_.compare("dubins") == 0)
            {
                const ob::DubinsStateSpace::StateType *state_r2;
                state_r2 = states[i - 1]->as<ob::DubinsStateSpace::StateType>();
                p.x = state_r2->getX();
                p.y = state_r2->getY();
            }
            else
            {
                const ob::RealVectorStateSpace::StateType *state_r2;
                state_r2 = states[i - 1]->as<ob::RealVectorStateSpace::StateType>();
                p.x = state_r2->values[0];
                p.y = state_r2->values[1];
            }

            p.z = 0.1;

            visual_result_path.points.push_back(p);
        }
    }
    solution_local_path_rviz_pub_->publish(visual_result_path);
}

bool OnlinePlannFramework::validateGoalCandidate(const ob::ScopedState<> &goal_candidate)
{
    if (simple_setup_local_->getStateValidityChecker()->isValid(goal_candidate->as<ob::State>()))
    {
        return true;
    }
    RCLCPP_WARN(this->get_logger(), "FIRST GOAL CANDIDATE INVALID!");
    return false;
}

ob::GoalStates *OnlinePlannFramework::findNewGoalCandidate(const ob::ScopedState<> &goal_candidate)
{

    ob::SpaceInformationPtr si_local = simple_setup_local_->getSpaceInformation();

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

        if (state_space_.compare("dubins") == 0)
        {
            new_goal_local[2] = goal_candidate[2];
        }

        if (simple_setup_local_->getStateValidityChecker()->isValid(new_goal_local->as<ob::State>()))
        {
            if (simple_setup_local_->getSpaceInformation()->checkMotion(goal_candidate->as<ob::State>(), new_goal_local->as<ob::State>()))
            {
                RCLCPP_WARN(this->get_logger(), "APPROXIMATE VALID GOAL FOUND!");

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
    RCLCPP_WARN(this->get_logger(), "NO OPTIONAL GOAL FOUND!");

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
    rclcpp::init(argc, argv);

    auto online_planning_framework = std::make_shared<OnlinePlannFramework>();
    online_planning_framework->planWithSimpleSetup();

    rclcpp::spin(online_planning_framework);
    return 0;
}

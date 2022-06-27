
#include "social_costmap.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <pedsim_msgs/AgentStates.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>

class WorldModeler
{
public:
    //! Constructor
    WorldModeler();
    //! Destructor
    virtual ~WorldModeler(){};

    // !CALLBACKS
    // ! global 2d map to be able to construct the costmap
    void global2DMapCallback(const nav_msgs::OccupancyGridConstPtr &map);
    // ! odometry of the robot
    void odometryCallback(const nav_msgs::OdometryConstPtr &odometry);
    // ! social agents states from the simulation
    void agentStatesCallback(const pedsim_msgs::AgentStatesPtr &social_agents);

    // ! PROCESSING FUNCTIONS
    pedsim_msgs::AgentStates socialAgentsInFOV();
    bool agentInFOV(pedsim_msgs::AgentState social_agent);

private:
    // ! PUBLISHERS
    ros::NodeHandle nh_, local_nh_;
    ros::Publisher social_costmap_pub_;

    // !SUBSCRIBERS
    ros::Subscriber projected_map_sub_, odometry_sub_, agent_states_sub_;

    // ! FRAMES/TOPICS
    std::string map_frame_,
        fixed_frame_, robot_frame_, odometry_topic_, global_2d_map_topic_, social_agents_topic_, social_costmap_topic_;

    bool add_rays_, apply_filter_, add_max_range_measures_, projection_2d_, global_map_available_;

    double octree_resol_, minimum_range_, rviz_timer_, robot_distance_view_, robot_velocity_thres_, robot_fov_;

    // Point Clouds
    std::vector<std::string> point_cloud_topics_, point_cloud_frames_;

    //  global map
    nav_msgs::OccupancyGrid global_2d_map;

    // odometry variable
    nav_msgs::Odometry robot_odometry;

    // agent states
    pedsim_msgs::AgentStates agent_states;

    // social costmap
    SocialCostmap *socialCostmap = new SocialCostmap();
    nav_msgs::OccupancyGrid current_social_costmap;
};

WorldModeler::WorldModeler()
    : nh_(),
      local_nh_("~"),
      fixed_frame_("/fixed_frame"),
      robot_frame_("/robot_frame"),
      odometry_topic_("/odometry_topic"),
      global_2d_map_topic_("/map"),
      octree_resol_(1.0),
      add_max_range_measures_(false),
      apply_filter_(false),
      projection_2d_(false),
      add_rays_(false),
      minimum_range_(-1.0),
      rviz_timer_(0.0)
{

    // ? PARAMETERS
    local_nh_.param("add_rays", add_rays_, add_rays_);
    local_nh_.param("resolution", octree_resol_, octree_resol_);
    local_nh_.param("apply_filter", apply_filter_, apply_filter_);
    local_nh_.param("add_max_ranges", add_max_range_measures_,
                    add_max_range_measures_);
    local_nh_.param("map_frame", map_frame_, map_frame_);
    local_nh_.param("fixed_frame", fixed_frame_, fixed_frame_);
    local_nh_.param("robot_frame", robot_frame_, robot_frame_);
    local_nh_.param("projection_2d", projection_2d_, projection_2d_);
    local_nh_.param("odometry_topic", odometry_topic_, odometry_topic_);
    local_nh_.param("global_2d_map_topic", global_2d_map_topic_, global_2d_map_topic_);
    local_nh_.param("minimum_range", minimum_range_, minimum_range_);
    local_nh_.param("rviz_timer", rviz_timer_, rviz_timer_);
    local_nh_.param("point_cloud_topics", point_cloud_topics_,
                    point_cloud_topics_);
    local_nh_.param("point_cloud_frames", point_cloud_frames_,
                    point_cloud_frames_);
    local_nh_.param("robot_distance_view", robot_distance_view_,
                    robot_distance_view_);
    local_nh_.param("robot_velocity_thres", robot_velocity_thres_,
                    robot_velocity_thres_);
    local_nh_.param("robot_fov", robot_fov_,
                    robot_fov_);
    local_nh_.param("social_agents_topic", social_agents_topic_,
                    social_agents_topic_);
    local_nh_.param("social_costmap_topic", social_costmap_topic_,
                    social_costmap_topic_);

    ros::Rate loop_rate(10);

    // !PUBLISHERS

    social_costmap_pub_ = local_nh_.advertise<nav_msgs::OccupancyGrid>(social_costmap_topic_, 1, true);

    // !SUBSCRIBERS

    odometry_sub_ = nh_.subscribe(odometry_topic_, 1,
                                  &WorldModeler::odometryCallback, this);

    agent_states_sub_ = nh_.subscribe(social_agents_topic_, 1,
                                      &WorldModeler::agentStatesCallback, this);

    // Global map
    projected_map_sub_ = nh_.subscribe(global_2d_map_topic_, 1,
                                       &WorldModeler::global2DMapCallback, this);
    global_map_available_ = false;
    if (!global_map_available_)
        ROS_WARN("%s:\n\tWaiting for global 2D map\n",
                 ros::this_node::getName().c_str());
    while (ros::ok() && !global_map_available_)
    {
        ROS_INFO_STREAM("inside while");
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO_STREAM("got global map");

    socialCostmap->setDimensions(global_2d_map.info.width, global_2d_map.info.height);
    ROS_INFO_STREAM("social costmap ready");
    socialCostmap->setFrameId(global_2d_map.header.frame_id);
    ROS_INFO_STREAM("social costmap ready");
    socialCostmap->setOrigin(global_2d_map.info.origin);
    ROS_INFO_STREAM("social costmap ready");
    socialCostmap->setResolution(global_2d_map.info.resolution);

    ROS_INFO_STREAM("social costmap ready");

    while (ros::ok())
    {
        ROS_INFO_STREAM("could get social agents in fov");
        pedsim_msgs::AgentStates social_agents_in_fov = socialAgentsInFOV();

        ROS_INFO_STREAM("could get social agents in fov");

        socialCostmap->updateSocialCostmap(global_2d_map.info.width, global_2d_map.info.height, global_2d_map.info.origin, &social_agents_in_fov);

        ROS_INFO_STREAM("social costmap updated");

        current_social_costmap = socialCostmap->getSocialCostmap();

        ROS_INFO_STREAM("data size normal costmap" << global_2d_map.data.size());

        ROS_INFO_STREAM("data size social costmap" << current_social_costmap.data.size());

        social_costmap_pub_.publish(current_social_costmap);

        ros::spinOnce();

        loop_rate.sleep();
    }
}

bool WorldModeler::agentInFOV(pedsim_msgs::AgentState social_agent)
{

    double dRobotAgent = std::sqrt(std::pow(social_agent.pose.position.x - robot_odometry.pose.pose.position.x, 2) +
                                   std::pow(social_agent.pose.position.y - robot_odometry.pose.pose.position.y, 2));

    double robotVelocity =
        std::sqrt(std::pow(robot_odometry.twist.twist.linear.x, 2) + std::pow(robot_odometry.twist.twist.linear.y, 2));

    double actualFOVDistance = robot_distance_view_ / robot_velocity_thres_ * robotVelocity;

    if (actualFOVDistance < 1.5)
    {
        actualFOVDistance = 1.5;
    }

    if (dRobotAgent > actualFOVDistance)
    {
        return false;
    }

    double tethaRobotAgent = atan2((social_agent.pose.position.y - robot_odometry.pose.pose.position.y),
                                   (social_agent.pose.position.x - robot_odometry.pose.pose.position.x));

    if (tethaRobotAgent < 0)
    {
        tethaRobotAgent = 2 * M_PI + tethaRobotAgent;
    }

    // ROS_INFO_STREAM("Angle robot agent: " << tethaRobotAgent);

    tf::Quaternion q(robot_odometry.pose.pose.orientation.x, robot_odometry.pose.pose.orientation.y,
                     robot_odometry.pose.pose.orientation.z, robot_odometry.pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double robotAngle = yaw;

    if (robotAngle < 0)
    {
        robotAngle = 2 * M_PI + robotAngle;
    }

    if (tethaRobotAgent > (robotAngle + M_PI))
        tethaRobotAgent = abs(robotAngle + 2 * M_PI - tethaRobotAgent);
    else if (robotAngle > (tethaRobotAgent + M_PI))
        tethaRobotAgent = abs(tethaRobotAgent + 2 * M_PI - robotAngle);
    else
        tethaRobotAgent = abs(tethaRobotAgent - robotAngle);

    if (abs(tethaRobotAgent) < robot_fov_)
        return true;

    return false;
}

pedsim_msgs::AgentStates WorldModeler::socialAgentsInFOV()
{

    pedsim_msgs::AgentStates agentStatesInFOV;
    std::vector<pedsim_msgs::AgentState> agentStatesVector;
    ROS_INFO_STREAM("running for agents in fov");
    for (int i = 0; i < agent_states.agent_states.size(); i++)
    {
        if (agentInFOV(agent_states.agent_states[i]))
        {
            ROS_INFO_STREAM("could gesocial agents in fov");
            agentStatesVector.push_back(agent_states.agent_states[i]);
        }
    }

    ROS_INFO_STREAM("agents in fov gotten");

    agentStatesInFOV.header = agent_states.header;
    agentStatesInFOV.agent_states = agentStatesVector;

    ROS_INFO_STREAM("about to return agents in fov");

    return agentStatesInFOV;
}

void WorldModeler::global2DMapCallback(const nav_msgs::OccupancyGridConstPtr &map)
{
    ROS_INFO_STREAM("en callback");
    global_map_available_ = true;
    global_2d_map.header = map->header;
    global_2d_map.data = map->data;
    global_2d_map.info = map->info;
    ROS_INFO_STREAM("ya con mapa");
}

void WorldModeler::odometryCallback(const nav_msgs::OdometryConstPtr &odometry)
{
    ROS_INFO_STREAM("in odom cb");
    robot_odometry.child_frame_id = odometry->child_frame_id;
    robot_odometry.header = odometry->header;
    robot_odometry.pose = odometry->pose;
    robot_odometry.twist = odometry->twist;
    ROS_INFO_STREAM("in odom cb finish");
}

void WorldModeler::agentStatesCallback(const pedsim_msgs::AgentStatesPtr &social_agents)
{
    ROS_INFO_STREAM("in agent state cb");
    agent_states.agent_states = social_agents->agent_states;
    agent_states.header = social_agents->header;
    ROS_INFO_STREAM("in agent state cb finish");
}

//! Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "world_modeler_node");

    ros::NodeHandle private_nh("~");

    WorldModeler world_modeler;

    ros::spin();

    return 0;
}
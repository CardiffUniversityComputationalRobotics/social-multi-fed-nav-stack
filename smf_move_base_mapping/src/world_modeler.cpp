
#include "social_costmap.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <pedsim_msgs/AgentStates.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>

// Octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

#include <tf2/LinearMath/Quaternion.h>

// FCL
#include <fcl/fcl.h>
#include <fcl/collision.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/broadphase/broadphase_spatialhash.h>
#include <fcl/common/types.h>
#include <fcl/config.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/math/geometry-inl.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

using octomap_msgs::GetOctomap;
// Standard namespace
using namespace std;
// Octomap namespace
using namespace octomap;

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
        fixed_frame_, robot_frame_, odometry_topic_, global_2d_map_topic_, social_agents_topic_, social_costmap_topic_, octomap_service_;

    bool add_rays_, apply_filter_, add_max_range_measures_, projection_2d_, global_map_available_;

    double octree_resol_, minimum_range_, rviz_timer_, robot_distance_view_, robot_velocity_thres_, robot_fov_, social_costmap_decay_factor_;

    int social_costmap_resolution_factor_;

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

    // OCTOMAP VARIABLES

    GetOctomap::Request req;
    GetOctomap::Response resp;

    octomap::AbstractOcTree *abs_octree_;
    octomap::OcTree *octree_;
    double octree_min_x_, octree_min_y_, octree_min_z_;
    double octree_max_x_, octree_max_y_, octree_max_z_;

    double octree_res_;

    // FCL
    fcl::OcTreef *tree_;
    fcl::CollisionObjectf *tree_obj_;
    std::shared_ptr<fcl::Boxf> robot_agent_solid_;
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
    local_nh_.param("social_costmap_decay_factor", social_costmap_decay_factor_,
                    social_costmap_decay_factor_);
    local_nh_.param("social_costmap_resolution_factor", social_costmap_resolution_factor_,
                    social_costmap_resolution_factor_);
    local_nh_.param("octomap_service", octomap_service_,
                    octomap_service_);

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
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_WARN("%s:\n\tGot global 2D map\n",
             ros::this_node::getName().c_str());

    socialCostmap->setResolutionFactor(social_costmap_resolution_factor_);
    socialCostmap->setDimensions(global_2d_map.info.width, global_2d_map.info.height);
    socialCostmap->setFrameId(global_2d_map.header.frame_id);
    socialCostmap->setOrigin(global_2d_map.info.origin);
    socialCostmap->setResolution(global_2d_map.info.resolution);
    socialCostmap->setTimeDecayFactor(social_costmap_decay_factor_);

    while (ros::ok())
    {
        pedsim_msgs::AgentStates social_agents_in_fov = socialAgentsInFOV();

        socialCostmap->updateSocialCostmap(global_2d_map.info.width, global_2d_map.info.height, global_2d_map.info.origin, &social_agents_in_fov);

        current_social_costmap = socialCostmap->getSocialCostmap();

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

    double actualFOVDistance = (robot_distance_view_ / robot_velocity_thres_) * robotVelocity;

    if (actualFOVDistance < 1.5)
    {
        actualFOVDistance = 1.5;
    }
    else if (actualFOVDistance > robot_distance_view_)
    {
        actualFOVDistance = robot_distance_view_;
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

    double tethaRobotAgentFov;

    if (tethaRobotAgent > (robotAngle + M_PI))
        tethaRobotAgentFov = abs(robotAngle + 2 * M_PI - tethaRobotAgent);
    else if (robotAngle > (tethaRobotAgent + M_PI))
        tethaRobotAgentFov = abs(tethaRobotAgent + 2 * M_PI - robotAngle);
    else
        tethaRobotAgentFov = abs(tethaRobotAgent - robotAngle);

    if (abs(tethaRobotAgentFov) < robot_fov_)
    {

        fcl::CollisionRequestf collision_request;
        fcl::CollisionResultf collision_result;

        robot_agent_solid_.reset(new fcl::Boxf(dRobotAgent, 0.2, 3));

        fcl::Transform3f robot_agent_solid_tf;
        robot_agent_solid_tf.setIdentity();
        robot_agent_solid_tf.translate(fcl::Vector3f((social_agent.pose.position.x + robot_odometry.pose.pose.position.x) / 2, (social_agent.pose.position.y + robot_odometry.pose.pose.position.y) / 2, 1));

        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, tethaRobotAgent, 0);

        robot_agent_solid_tf.rotate(fcl::Quaternionf(myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW()));

        fcl::CollisionObjectf robot_agent_co(robot_agent_solid_, robot_agent_solid_tf);

        fcl::collide(tree_obj_, &robot_agent_co, collision_request, collision_result);

        if (collision_result.isCollision())
        {
            return false;
        }
        return true;
    }
    return false;
}

pedsim_msgs::AgentStates WorldModeler::socialAgentsInFOV()
{

    pedsim_msgs::AgentStates agentStatesInFOV;
    std::vector<pedsim_msgs::AgentState> agentStatesVector;

    while ((nh_.ok() && !ros::service::call(octomap_service_, req, resp)) || resp.map.data.size() == 0)
    {
        ROS_WARN("Requestt to %s failed; trying again...", nh_.resolveName(octomap_service_).c_str());
        usleep(1000000);
    }
    if (nh_.ok())
    { // skip when CTRL-C
        abs_octree_ = octomap_msgs::msgToMap(resp.map);
        std::cout << std::endl;
        if (abs_octree_)
        {
            octree_ = dynamic_cast<octomap::OcTree *>(abs_octree_);
            tree_ = new fcl::OcTreef(std::shared_ptr<const octomap::OcTree>(octree_));
            tree_obj_ = new fcl::CollisionObjectf((std::shared_ptr<fcl::CollisionGeometryf>(tree_)));
        }
    }

    for (int i = 0; i < agent_states.agent_states.size(); i++)
    {
        if (agentInFOV(agent_states.agent_states[i]))
        {
            agentStatesVector.push_back(agent_states.agent_states[i]);
        }
    }

    agentStatesInFOV.header = agent_states.header;
    agentStatesInFOV.agent_states = agentStatesVector;

    return agentStatesInFOV;
}

void WorldModeler::global2DMapCallback(const nav_msgs::OccupancyGridConstPtr &map)
{
    global_map_available_ = true;
    global_2d_map.header = map->header;
    global_2d_map.data = map->data;
    global_2d_map.info = map->info;
}

void WorldModeler::odometryCallback(const nav_msgs::OdometryConstPtr &odometry)
{
    robot_odometry.child_frame_id = odometry->child_frame_id;
    robot_odometry.header = odometry->header;
    robot_odometry.pose = odometry->pose;
    robot_odometry.twist = odometry->twist;
}

void WorldModeler::agentStatesCallback(const pedsim_msgs::AgentStatesPtr &social_agents)
{
    agent_states.agent_states = social_agents->agent_states;
    agent_states.header = social_agents->header;
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
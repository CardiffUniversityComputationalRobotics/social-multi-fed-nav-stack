
#include "social_costmap.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <pedsim_msgs/AgentStates.h>
#include <nav_msgs/OccupancyGrid.h>

class WorldModeler
{
public:
    //! Constructor
    WorldModeler();
    //! Destructor
    virtual ~WorldModeler(){};

    void global2DMapCallback(const nav_msgs::OccupancyGridConstPtr &global_2d_map);

private:
    // ! ROS OBJECTS SUBSCRIBERS PUBLISHERS
    ros::NodeHandle nh_, local_nh_;
    ros::Publisher social_costmap_pub_;
    ros::Subscriber projected_map_sub_;

    // ! FRAMES/TOPICS
    std::string map_frame_,
        fixed_frame_, robot_frame_, odometry_topic_, global_2d_map_topic_;

    bool add_rays_, apply_filter_, add_max_range_measures_, projection_2d_, global_map_available_;

    double octree_resol_, minimum_range_, rviz_timer_;

    // Point Clouds
    std::vector<std::string> point_cloud_topics_, point_cloud_frames_;

    // social costmap
    SocialCostmap *socialCostmap;
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

    ros::Rate loop_rate(10);

    // Global map
    projected_map_sub_ = nh_.subscribe(global_2d_map_topic_, 1,
                                       &WorldModeler::global2DMapCallback, this);
    global_map_available_ = false;
    if (!global_map_available_)
        ROS_WARN("%s:\n\tWaiting for global map\n",
                 ros::this_node::getName().c_str());
    while (ros::ok() && !global_map_available_)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void WorldModeler::global2DMapCallback(const nav_msgs::OccupancyGridConstPtr &global_2d_map)
{
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
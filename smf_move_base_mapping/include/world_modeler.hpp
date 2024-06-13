// SOCIAL HEATMAP
#include "social_heatmap.hpp"

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// ROS LaserScan tools
#include <laser_geometry/laser_geometry.h>
#include <cmath>
// ROS messages
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/MarkerArray.h>

// ROS services
#include <std_srvs/Empty.h>

// ROS tf
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

// Octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
typedef octomap_msgs::GetOctomap OctomapSrv;
#include <octomap/Pointcloud.h>
#include <octomap_ros/conversions.h>

// grid map library
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_octomap/grid_map_octomap.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <sensor_msgs/PointCloud2.h>

// PEDSIM
#include <pedsim_msgs/AgentStates.h>

#include <signal.h>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
typedef octomap::OcTree OcTreeT;

void stopNode(int sig)
{
    ros::shutdown();
    exit(0);
}

//!  WorldModeler class.
/*!
 * Autopilot Laser WorldModeler.
 * Create an WorldModeler using information from laser scans.
 */
class WorldModeler
{
public:
    //! Constructor
    WorldModeler();
    //! Destructor
    virtual ~WorldModeler();
    //! Callback for getting the point_cloud data
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);
    //! Callback for getting current vehicle odometry
    void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
    //! Callback for getting current agent states
    void agentStatesCallback(const pedsim_msgs::AgentStatesConstPtr &agent_states_msg);
    bool isAgentInRFOV(const pedsim_msgs::AgentState agent_state);
    // ! Check if robot is in front of agent.
    bool isRobotInFront(pedsim_msgs::AgentState agent_state, grid_map::Position position);
    // ! Calculate comfort in specific position in gridmap
    double getExtendedPersonalSpace(pedsim_msgs::AgentState agent_state, grid_map::Position position);
    //! Periodic callback to publish the map for visualization.
    void timerCallback(const ros::TimerEvent &e);
    void insertScan(const tf::Point &sensorOriginTf, const PCLPointCloud &ground,
                    const PCLPointCloud &nonground);
    //! Service to save a binary Octomap (.bt)
    bool saveBinaryOctomapSrv(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res);
    //! Service to save a full Octomap (.ot)
    bool saveFullOctomapSrv(std_srvs::Empty::Request &req,
                            std_srvs::Empty::Response &res);
    //! Service to get a binary Octomap
    bool getBinaryOctomapSrv(OctomapSrv::Request &req,
                             OctomapSrv::GetOctomap::Response &res);
    //! Service to get grid map
    bool getGridMapSrv(grid_map_msgs::GetGridMap::Request &req,
                       grid_map_msgs::GetGridMap::Response &res);

    //! Publish the WorldModeler
    void publishMap();
    //! Redefine the gridmap with octomap
    void defineSocialGridMap();

private:
    // ROS
    ros::NodeHandle nh_, local_nh_;
    ros::Publisher octomap_marker_pub_, grid_map_pub_, relevant_agents_pub_;
    ros::Subscriber odom_sub_, agent_states_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
    ros::ServiceServer save_binary_octomap_srv_, save_full_octomap_srv_,
        get_binary_octomap_srv_, get_grid_map_srv_;
    ros::Timer timer_;
    boost::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>> point_cloud_mn_;

    // ROS tf
    tf::TransformListener tf_listener_;

    // Names
    std::string map_frame_, fixed_frame_, robot_frame_, offline_octomap_path_,
        odometry_topic_, social_agents_topic_;

    // Point Clouds
    std::string point_cloud_topic_, point_cloud_frame_;

    // ROS Messages
    sensor_msgs::PointCloud cloud_;

    nav_msgs::OdometryConstPtr robot_odometry_;

    SocialHeatmap social_heatmap_ = SocialHeatmap();

    // pedsim messages
    pedsim_msgs::AgentStatesConstPtr agent_states_;
    pedsim_msgs::AgentStates relevant_agent_states_;

    pedsim_msgs::AgentStates social_agents_in_radius_;
    std::vector<pedsim_msgs::AgentState> social_agents_in_radius_vector_;

    double social_agent_radius_;

    // Octree
    octomap::OcTree *octree_;
    double octree_resol_, rviz_timer_;
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;
    octomap::KeyRay m_keyRay; // temp storage for ray casting
    double mapping_max_range_, min_z_pc_, max_z_pc_;

    // grid map
    grid_map::GridMap grid_map_;

    grid_map_msgs::GridMap grid_map_msg_;

    // social relevance validity checking constants
    double robot_distance_view_max_, robot_distance_view_min_, robot_velocity_threshold_, robot_angle_view_, actual_fov_distance_;

    double social_heatmap_decay_factor_;

    //! basic social personal space parameters defined
    /*
     * amplitude of basic social personal space function
     */
    double social_comfort_amplitude_ = 4;

    /*
     * standard deviation in X of gaussian basic social personal space function
     */
    double sigma_x = 0.45;

    /*
     * standard deviation in X of gaussian basic social personal space function
     */
    double sigma_y = 0.45;

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

    // Flags
    bool initialized_;
    bool nav_sts_available_;
    bool visualize_free_space_;
    bool social_relevance_validity_checking_;

protected:
    inline static void updateMinKey(const octomap::OcTreeKey &in,
                                    octomap::OcTreeKey &min)
    {
        for (unsigned i = 0; i < 3; ++i)
            min[i] = std::min(in[i], min[i]);
    };
    inline static void updateMaxKey(const octomap::OcTreeKey &in,
                                    octomap::OcTreeKey &max)
    {
        for (unsigned i = 0; i < 3; ++i)
            max[i] = std::max(in[i], max[i]);
    };
};

#include <nav_msgs/OccupancyGrid.h>
#include <social_costmap.h>

//! CONSTRUCTOR
SocialCostmap::SocialCostmap(std::string frameId, unsigned int width, unsigned int height, geometry_msgs::Pose origin){

}

//!FUNCTIONS
void SocialCostmap::setDimensions(unsigned int width, unsigned int height){

}


//! Main function
int main(int argc, char **argv)
{
  //=======================================================================
  // Override SIGINT handler
  //=======================================================================
//   signal(SIGINT, stopNode);

  // Init ROS node
//   ros::init(argc, argv, "octomap_laser_scan");
//   ros::NodeHandle private_nh("~");

//   // Constructor
//   LaserOctomap mapper;

//   // Spin
//   ros::spin();

  // Exit main function without errors
  return 0;
}
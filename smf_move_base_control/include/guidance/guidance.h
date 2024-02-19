#ifndef OTTER_GUIDANCE_H_
#define OTTER_GUIDANCE_H_

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <smf_move_base_msgs/Path2D.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>

namespace otter_coverage
{

  class Guidance
  {
  public:
    Guidance();
    ~Guidance();

  private:
    void newWaypoint(const geometry_msgs::PoseStamped &waypoint);
    void newPath(const nav_msgs::Path &path);
    void followPath(double x, double y, double psi);
    double dist(double x0, double y0, double x1, double y1) const;

    nav_msgs::Path m_path;

    ros::Publisher m_controllerPub;

    // lookahead distance
    double DELTA = 0.2;

    // time-varying lookahead distance
    double delta_max = 1.5;
    double delta_min = 0.3;
    double delta_k = 0.3;

    // circle of acceptance
    double R = 0.3;

    double m_maxSpeed;
    double m_maxSpeedTurn;
    double m_minSpeed;
  };

} // namespace otter_coverage

#endif

#include <guidance/guidance.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>

namespace otter_coverage
{

  Guidance::Guidance()
  {
    ros::NodeHandle nh;
    ros::NodeHandle nhP("~");

    m_maxSpeed = nhP.param("max_speed", 0.4);
    m_maxSpeedTurn = nhP.param("max_speed_turn", 0.6);
    m_minSpeed = nhP.param("min_speed", 0.0);

    ros::Subscriber dubinsPathSub =
        nh.subscribe("/smf_move_base_planner/path", 1000, &Guidance::newPath, this);

    m_controllerPub =
        nh.advertise<geometry_msgs::Twist>("/stretch_diff_drive_controller/cmd_vel", 1000);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);
    while (nh.ok())
    {
      // Get the pose of the robot in the map frame
      geometry_msgs::TransformStamped tfStamped;
      try
      {
        tfStamped = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0.0),
                                             ros::Duration(0.0));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("Transform from map to base_link not found: %s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
      double x = tfStamped.transform.translation.x;
      double y = tfStamped.transform.translation.y;
      double psi = tf2::getYaw(tfStamped.transform.rotation);

      followPath(x, y, psi);

      ros::spinOnce();
      rate.sleep();
    }
  }

  Guidance::~Guidance() {}

  void Guidance::newPath(const nav_msgs::Path &path)
  {

    m_path = path;
    ROS_INFO_STREAM("RECEIVING PATH");
  }

  void Guidance::followPath(double x, double y, double psi)
  // TODO: cuts turns, how to fix?
  {

    ROS_INFO_STREAM("STARTING CONTROLLER");

    // Finished?
    if (m_path.poses.size() <= 1)
    {
      ROS_INFO_STREAM("WAYPOINTS SIZE:" << m_path.poses.size());
      geometry_msgs::Twist msg;
      msg.linear.x = 0;
      msg.angular.z = 0;
      m_controllerPub.publish(msg);
      return;
    }

    // Identify closest point on path
    std::vector<geometry_msgs::PoseStamped>::iterator closest;
    double minDist = std::numeric_limits<double>::max();
    for (auto it = m_path.poses.begin(); it != m_path.poses.end(); it++)
    {
      double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) +
                              std::pow(y - it->pose.position.y, 2));
      if (dist < minDist)
      {
        minDist = dist;
        closest = it;
      }
    }

    // Store closest
    geometry_msgs::PoseStamped pose_d;
    pose_d.pose.position.x = closest->pose.position.x;
    pose_d.pose.position.y = closest->pose.position.y;

    // tf2::Quaternion pose_quaternion;
    // pose_quaternion.setRPY(0, 0, closest->theta);
    // pose_quaternion = pose_quaternion.normalize();

    pose_d.pose.orientation.x = closest->pose.orientation.x;
    pose_d.pose.orientation.x = closest->pose.orientation.y;
    pose_d.pose.orientation.x = closest->pose.orientation.z;
    pose_d.pose.orientation.x = closest->pose.orientation.w;

    // Erase previous elements
    m_path.poses.erase(m_path.poses.begin(), closest);

    // Path tangential angle
    double gamma_p = tf2::getYaw(pose_d.pose.orientation);

    // Cross-track error
    double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) +
                 (y - pose_d.pose.position.y) * std::cos(gamma_p);

    // Time-varying lookahead distance
    double delta_y_e =
        (delta_max - delta_min) * std::exp(-delta_k * std::pow(y_e, 2)) +
        delta_min;
    // if turning => small lookahead distance
    bool isTurning = false;
    if ((closest + 1) != m_path.poses.end())
    {
      tf::Quaternion q((*(closest + 1)).pose.orientation.x, (*(closest + 1)).pose.orientation.y, (*(closest + 1)).pose.orientation.z, (*(closest + 1)).pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      double nextAngle = yaw;
      // ROS_INFO_STREAM("ROBOT YAW:: " << yaw)
      if (std::fabs(gamma_p - nextAngle) > std::numeric_limits<double>::epsilon())
      {
        delta_y_e = delta_min;
        isTurning = true;
      }
    }

    // velocity-path relative angle
    double chi_r = std::atan(-y_e / delta_y_e);

    // desired course angle
    double chi_d = gamma_p + chi_r;

    ROS_INFO_STREAM("CHI_RR:: " << chi_d);

    // calculate error in heading
    double chi_err = chi_d - psi;
    while (chi_err > M_PI)
    {
      chi_err -= 2 * M_PI;
    }
    while (chi_err < -M_PI)
    {
      chi_err += 2 * M_PI;
    }

    // calculate desired speed
    ROS_INFO_STREAM("Y_E::: " << y_e);
    ROS_INFO_STREAM("CHI_ERR::: " << chi_err);
    double u = m_maxSpeed * (1 - std::abs(y_e) / 5 - std::abs(chi_err) / M_PI_2);
    u = std::max(u, m_minSpeed);
    if (isTurning)
      u = m_maxSpeedTurn;

    // Publish speed and course to controller
    geometry_msgs::Twist msg;
    msg.linear.x = u;
    msg.angular.z = chi_d;
    // m_controllerPub.publish(msg);

    ROS_INFO_STREAM("psi_d: " << chi_d << " psi: " << psi);
    ROS_INFO_STREAM("u_d: " << u);
  }

} // namespace otter_coverage


#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <cstdlib>
#include <cmath>
#include <string>
#include <geometry_msgs/Pose.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <map>
#include <smf_move_base_msgs/RelevantAgentState.h>

// grid map library
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_octomap/grid_map_octomap.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/tf.h>

class SocialHeatmap
{
private:
    double time_decay_factor_ = 1;

    unsigned int resolution_factor_ = 5;

    std::map<unsigned int, smf_move_base_msgs::RelevantAgentState> agent_states_record_;

    grid_map::GridMap social_heatmap_;

public:
    //! CONSTRUCTOR

    SocialHeatmap();

    SocialHeatmap(grid_map::GridMap grid_map);

    //! FUNCTIONS

    void updateSocialHeatmap(grid_map::GridMap grid_map, pedsim_msgs::AgentStates agentStates);

    void addNewAgentStates(pedsim_msgs::AgentStates agent_states);

    void updateAgentStatesRelevance(pedsim_msgs::AgentStates agent_states);

    //! GETTERS
    grid_map::Matrix getSocialHeatmap()
    {
        return social_heatmap_["social_heatmap"];
    }

    //! SETTERS

    void setTimeDecayFactor(double time_decay_factor);

    // ! EXTERNAL FUNCTIONS

    double socialComfortCost(double x, double y, smf_move_base_msgs::RelevantAgentState relevant_agent_state)
    {
        double distance = std::sqrt(std::pow(relevant_agent_state.agent_state.pose.position.x - x, 2) +
                                    std::pow(relevant_agent_state.agent_state.pose.position.y - y, 2));

        double tetha_robot_agent = atan2((y - relevant_agent_state.agent_state.pose.position.y),
                                         (x - relevant_agent_state.agent_state.pose.position.x));

        if (tetha_robot_agent < 0)
        {
            tetha_robot_agent = 2 * M_PI + tetha_robot_agent;
        }

        double tetha_orientation;
        if (abs(relevant_agent_state.agent_state.twist.linear.x) > 0 || abs(relevant_agent_state.agent_state.twist.linear.y) > 0)
        {
            tetha_orientation = atan2(relevant_agent_state.agent_state.twist.linear.y, relevant_agent_state.agent_state.twist.linear.x);
        }
        else
        {
            tf::Quaternion q(relevant_agent_state.agent_state.pose.orientation.x, relevant_agent_state.agent_state.pose.orientation.y,
                             relevant_agent_state.agent_state.pose.orientation.z, relevant_agent_state.agent_state.pose.orientation.w);

            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            tetha_orientation = yaw;
        }

        if (tetha_orientation < 0)
        {
            tetha_orientation = 2 * M_PI + tetha_orientation;
        }

        double basic_personal_space_val =
            relevant_agent_state.relevance *
            std::exp(-(
                std::pow(distance * std::cos(tetha_robot_agent - tetha_orientation) / (std::sqrt(2) * 0.45),
                         2) +
                std::pow(distance * std::sin(tetha_robot_agent - tetha_orientation) / (std::sqrt(2) * 0.45),
                         2)));

        return basic_personal_space_val;
    }
};


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

#include <tf/tf.h>

class SocialCostmap
{
private:
    nav_msgs::OccupancyGrid social_costmap_;

    unsigned int lastUpdateTime_;
    unsigned int width_;
    unsigned int height_;
    double resolution_;
    geometry_msgs::Pose origin_;
    std::string frameId_;

    double time_decay_factor_ = 1;

    unsigned int resolution_factor_ = 5;

    std::map<unsigned int, smf_move_base_msgs::RelevantAgentState> agent_states_record_;

public:
    //! CONSTRUCTOR

    SocialCostmap();

    SocialCostmap(std::string frameId, unsigned int width, unsigned int height, geometry_msgs::Pose origin, double resolution);

    //! FUNCTIONS

    unsigned int calculateSocialCost(double x, double y);

    void updateSocialCostmap(unsigned int width, unsigned int height, geometry_msgs::Pose origin, pedsim_msgs::AgentStates agentStates);

    void initSocialCostmap();

    void addNewAgentStates(pedsim_msgs::AgentStates agentStates);

    void updateAgentStatesRelevance(pedsim_msgs::AgentStates agentStates);

    //! GETTERS
    nav_msgs::OccupancyGrid getSocialCostmap()
    {
        return social_costmap_;
    }

    //! SETTERS
    void setDimensions(unsigned int width, unsigned int height);

    void setResolution(double resolution);

    void setOrigin(geometry_msgs::Pose origin);

    void setTimeDecayFactor(double timeDecayFactor);

    void setFrameId(std::string frameId);

    void setResolutionFactor(unsigned int resolutionFactor);

    // ! EXTERNAL FUNCTIONS

    unsigned int mapIndex(unsigned int width, unsigned int i, unsigned int j)
    {
        return i + j * width;
    }

    double mapWx(double origin_x, unsigned int width, double resolution, unsigned int i)
    {

        return double(origin_x) + (double(i) - double(width) / 2) * double(resolution);
    }

    double mapWy(double origin_y, unsigned int height, double resolution, unsigned int j)
    {
        return double(origin_y) + (double(j) - double(height) / 2) * double(resolution);
    }

    double socialComfortCost(double x, double y, smf_move_base_msgs::RelevantAgentState relevantAgentState)
    {
        double distance = std::sqrt(std::pow(relevantAgentState.agent_state.pose.position.x - x, 2) +
                                    std::pow(relevantAgentState.agent_state.pose.position.y - y, 2));

        double tethaRobotAgent = atan2((y - relevantAgentState.agent_state.pose.position.y),
                                       (x - relevantAgentState.agent_state.pose.position.x));

        if (tethaRobotAgent < 0)
        {
            tethaRobotAgent = 2 * M_PI + tethaRobotAgent;
        }

        double tethaOrientation;
        if (abs(relevantAgentState.agent_state.twist.linear.x) > 0 || abs(relevantAgentState.agent_state.twist.linear.y) > 0)
        {
            tethaOrientation = atan2(relevantAgentState.agent_state.twist.linear.y, relevantAgentState.agent_state.twist.linear.x);
        }
        else
        {
            tf::Quaternion q(relevantAgentState.agent_state.pose.orientation.x, relevantAgentState.agent_state.pose.orientation.y,
                             relevantAgentState.agent_state.pose.orientation.z, relevantAgentState.agent_state.pose.orientation.w);

            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            tethaOrientation = yaw;
        }

        if (tethaOrientation < 0)
        {
            tethaOrientation = 2 * M_PI + tethaOrientation;
        }

        double basicPersonalSpaceVal =
            relevantAgentState.relevance *
            std::exp(-(
                std::pow(distance * std::cos(tethaRobotAgent - tethaOrientation) / (std::sqrt(2) * 0.45),
                         2) +
                std::pow(distance * std::sin(tethaRobotAgent - tethaOrientation) / (std::sqrt(2) * 0.45),
                         2)));

        return basicPersonalSpaceVal;
    }
};

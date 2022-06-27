
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
    nav_msgs::OccupancyGrid socialCostmap;

    unsigned int lastUpdateTime;
    unsigned int width;
    unsigned int height;
    float resolution;
    geometry_msgs::Pose origin;
    std::string frameId;

    unsigned int timeDecayFactor = 1;

    std::map<unsigned int, smf_move_base_msgs::RelevantAgentState> agentStatesRecord;

public:
    //! CONSTRUCTOR

    SocialCostmap();

    SocialCostmap(std::string frameId, unsigned int width, unsigned int height, geometry_msgs::Pose origin, float resolution);

    //! FUNCTIONS

    unsigned int calculateSocialCost(float x, float y);

    void updateSocialCostmap(unsigned int width, unsigned int height, geometry_msgs::Pose origin, pedsim_msgs::AgentStates *agentStates);

    void initSocialCostmap();

    void addNewAgentStates(pedsim_msgs::AgentStates *agentStates);

    void updateAgentStatesRelevance(pedsim_msgs::AgentStates *agentStates);

    //! GETTERS
    nav_msgs::OccupancyGrid getSocialCostmap()
    {
        return socialCostmap;
    }

    //! SETTERS
    void setDimensions(unsigned int width, unsigned int height);

    void setResolution(float resolution);

    void setOrigin(geometry_msgs::Pose origin);

    void setTimeDecayFactor(unsigned int timeDecayFactor);

    void setFrameId(std::string);

    // ! EXTERNAL FUNCTIONS

    unsigned int mapIndex(unsigned int width, unsigned int i, unsigned int j)
    {
        return i + j * width;
    }

    float mapWx(float origin_x, unsigned int width, float resolution, unsigned int i)
    {
        return origin_x + (i - width / 2) * resolution;
    }

    float mapWy(float origin_y, unsigned int height, float resolution, unsigned int i)
    {
        return origin_y + (i - height / 2) * resolution;
    }

    unsigned int socialComfortCost(float x, float y, smf_move_base_msgs::RelevantAgentState relevantAgentState)
    {
        float distance = std::sqrt(std::pow(relevantAgentState.agent_state.pose.position.x - x, 2) +
                                   std::pow(relevantAgentState.agent_state.pose.position.y - y, 2));

        float tethaRobotAgent = atan2((y - relevantAgentState.agent_state.pose.position.y),
                                      (x - relevantAgentState.agent_state.pose.position.x));

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

        unsigned int basicPersonalSpaceVal =
            relevantAgentState.relevance *
            std::exp(-(
                std::pow(distance * std::cos(tethaRobotAgent - tethaOrientation) / (std::sqrt(2) * 0.45),
                         2) +
                std::pow(distance * std::cos(tethaRobotAgent - tethaOrientation) / (std::sqrt(2) * 0.45),
                         2)));
        return basicPersonalSpaceVal;
    }
};

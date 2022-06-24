
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

    int mapIndex(unsigned int width, unsigned int i, unsigned int j)
    {
        return i + j * width;
    }

    float mapWx(float origin_x, unsigned int width, float resolution, int i)
    {
        return origin_x + (i - width / 2) * resolution;
    }

    float mapWy(float origin_y, unsigned int height, float resolution, int i)
    {
        return origin_y + (i - height / 2) * resolution;
    }
};


#include <nav_msgs/OccupancyGrid.h>
#include <cstdlib>
#include <cmath>
#include <string>
#include <geometry_msgs/Pose.h>
#include <pedsim_msgs/AgentStates.h>
#include <map>
#include <smf_move_base_msgs/RelevantAgentState.h>


class SocialCostmap{
    private:
        nav_msgs::OccupancyGrid socialCostmap;

        unsigned int lastUpdateTime;
        unsigned int width;
        unsigned int height;
        geometry_msgs::Pose origin;

        unsigned int timeDecayFactor = 1;

        std::map<unsigned int, smf_move_base_msgs::RelevantAgentState> agentStatesRecord;

    public: 
        SocialCostmap(std::string frameId, unsigned int width, unsigned int height, geometry_msgs::Pose origin, float resolution);

        void updateSocialCostmap(unsigned int width, unsigned int height, geometry_msgs::Pose origin, pedsim_msgs::AgentStates agentStates);


        //? GETTERS
        nav_msgs::OccupancyGrid getSocialCostmap(){
            return socialCostmap;
        }

        //? SETTERS
        void setDimensions(unsigned int width, unsigned int height);

        void setTimeDecayFactor(unsigned int timeDecayFactor);
};


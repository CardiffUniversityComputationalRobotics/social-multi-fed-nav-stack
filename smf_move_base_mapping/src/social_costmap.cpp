
#include <social_costmap.h>

//! CONSTRUCTOR
SocialCostmap::SocialCostmap()
{
    setDimensions(1, 1);
    setOrigin(geometry_msgs::Pose());
    setResolution(0.5);
    setFrameId("world");
    initSocialCostmap();
}

SocialCostmap::SocialCostmap(std::string frameId, unsigned int width, unsigned int height, geometry_msgs::Pose origin, double resolution)
{

    setDimensions(width, height);
    setOrigin(origin);
    setResolution(resolution);
    setFrameId(frameId);
    initSocialCostmap();
}

//! FUNCTIONS
void SocialCostmap::updateSocialCostmap(unsigned int width, unsigned int height, geometry_msgs::Pose origin, pedsim_msgs::AgentStates agentStates)
{
    setDimensions(width, height);
    setOrigin(origin);

    updateAgentStatesRelevance(agentStates);

    addNewAgentStates(agentStates);

    // header
    social_costmap_.header.stamp = ros::Time::now();
    social_costmap_.header.frame_id = frameId_;

    // info
    social_costmap_.info.resolution = resolution_;
    social_costmap_.info.width = width_;
    social_costmap_.info.height = height_;
    social_costmap_.info.origin = origin_;

    int dataArraySize = width_ * height_;

    social_costmap_.data.resize(dataArraySize);

    double mapOriginX = origin_.position.x + (width_ / 2) * resolution_;

    double mapOriginY = origin_.position.y + (height_ / 2) * resolution_;

    for (int j = 0; j < height_; j++)
    {
        for (int i = 0; i < width_; i++)
        {
            double wX = mapWx(mapOriginX, width_, resolution_, i);
            double wY = mapWy(mapOriginY, height_, resolution_, j);

            social_costmap_.data[mapIndex(width_, i, j)] = calculateSocialCost(wX, wY);
        }
    }
}

unsigned int SocialCostmap::calculateSocialCost(double x, double y)
{

    double socialCost = 0;

    for (auto &agentItem : agent_states_record_)
    {
        auto &relevantAgentState = agentItem.second;
        socialCost += socialComfortCost(x, y, relevantAgentState);
    }

    if (socialCost > 100)
    {
        socialCost = 100;
    }

    return int(socialCost);
}

void SocialCostmap::updateAgentStatesRelevance(pedsim_msgs::AgentStates agentStates)
{

    std::vector<int> irrelevant_agents;

    for (auto &agentItem : agent_states_record_)
    {
        if (agentItem.second.relevance <= 0)
        {
            irrelevant_agents.push_back(agentItem.first);
        }
    }

    // ROS_INFO_STREAM("number of irrelevant agents: " << irrelevant_agents.size());

    for (int i = 0; i < irrelevant_agents.size(); i++)
    {
        agent_states_record_.erase(irrelevant_agents[i]);
    }
    // ROS_INFO_STREAM("number of agents recorded: " << agent_states_record_.size());

    for (auto &agentItem : agent_states_record_)
    {
        auto &relevantAgentState = agentItem.second;

        smf_move_base_msgs::RelevantAgentState newRelevantAgentState;
        newRelevantAgentState.header = relevantAgentState.header;
        newRelevantAgentState.agent_state = relevantAgentState.agent_state;

        newRelevantAgentState.relevance = relevantAgentState.relevance - double(time_decay_factor_ * exp(double(time_decay_factor_ * time_decay_factor_ * (ros::Time::now().sec - relevantAgentState.agent_state.header.stamp.now().sec))));

        if (newRelevantAgentState.relevance < 0)
        {
            newRelevantAgentState.relevance = 0;
        }

        agentItem.second = newRelevantAgentState;
    }
}

void SocialCostmap::addNewAgentStates(pedsim_msgs::AgentStates agentStates)
{

    for (int i = 0; i < agentStates.agent_states.size(); i++)
    {

        smf_move_base_msgs::RelevantAgentState relevantAgentState;

        relevantAgentState.header = agentStates.header;

        relevantAgentState.agent_state = agentStates.agent_states[i];

        relevantAgentState.relevance = 100;

        agent_states_record_[agentStates.agent_states[i].id] = relevantAgentState;
    }
}

void SocialCostmap::initSocialCostmap()
{
    // social costmap initialization

    // header
    social_costmap_.header.stamp = ros::Time::now();
    social_costmap_.header.frame_id = frameId_;

    // info
    social_costmap_.info.resolution = resolution_;
    social_costmap_.info.width = width_;
    social_costmap_.info.height = height_;
    social_costmap_.info.origin = origin_;

    int dataArraySize = width_ * height_;

    social_costmap_.data.resize(dataArraySize);

    lastUpdateTime_ = ros::Time::now().sec;
}

// ! SETTERS
void SocialCostmap::setDimensions(unsigned int width, unsigned int height)
{
    width_ = int(width / resolution_factor_);
    height_ = int(height / resolution_factor_);
}

void SocialCostmap::setOrigin(geometry_msgs::Pose origin)
{
    origin_ = origin;
}

void SocialCostmap::setTimeDecayFactor(double timeDecayFactor)
{
    time_decay_factor_ = timeDecayFactor;
}

void SocialCostmap::setResolution(double resolution)
{
    resolution_ = resolution * resolution_factor_;
}

void SocialCostmap::setFrameId(std::string frameId)
{
    frameId_ = frameId;
}

void SocialCostmap::setResolutionFactor(unsigned int resolutionFactor)
{
    resolution_factor_ = resolutionFactor;
}

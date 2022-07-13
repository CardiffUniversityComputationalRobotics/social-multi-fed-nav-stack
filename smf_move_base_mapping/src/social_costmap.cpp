
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
void SocialCostmap::updateSocialCostmap(unsigned int width, unsigned int height, geometry_msgs::Pose origin, pedsim_msgs::AgentStates *agentStates)
{
    setDimensions(width, height);
    setOrigin(origin);

    updateAgentStatesRelevance(agentStates);

    addNewAgentStates(agentStates);

    // header
    this->socialCostmap.header.stamp = ros::Time::now();
    this->socialCostmap.header.frame_id = this->frameId_;

    // info
    this->socialCostmap.info.resolution = this->resolution_;
    this->socialCostmap.info.width = this->width_;
    this->socialCostmap.info.height = this->height_;
    this->socialCostmap.info.origin = this->origin_;

    int dataArraySize = this->width_ * this->height_;

    this->socialCostmap.data.resize(dataArraySize);

    double mapOriginX = this->origin_.position.x + (this->width_ / 2) * this->resolution_;

    double mapOriginY = this->origin_.position.y + (this->height_ / 2) * this->resolution_;

    for (int j = 0; j < this->height_; j++)
    {
        for (int i = 0; i < this->width_; i++)
        {
            double wX = mapWx(mapOriginX, this->width_, this->resolution_, i);
            double wY = mapWy(mapOriginY, this->height_, this->resolution_, j);

            this->socialCostmap.data[mapIndex(this->width_, i, j)] = this->calculateSocialCost(wX, wY);
        }
    }

    free(agentStates);
}

unsigned int SocialCostmap::calculateSocialCost(double x, double y)
{

    double socialCost = 0;

    for (auto &agentItem : this->agentStatesRecord)
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

void SocialCostmap::updateAgentStatesRelevance(pedsim_msgs::AgentStates *agentStates)
{

    std::vector<int> irrelevant_agents;

    for (auto &agentItem : this->agentStatesRecord)
    {
        if (agentItem.second.relevance <= 0)
        {
            irrelevant_agents.push_back(agentItem.first);
        }
    }

    // ROS_INFO_STREAM("number of irrelevant agents: " << irrelevant_agents.size());

    for (int i = 0; i < irrelevant_agents.size(); i++)
    {
        this->agentStatesRecord.erase(irrelevant_agents[i]);
    }
    // ROS_INFO_STREAM("number of agents recorded: " << this->agentStatesRecord.size());

    for (auto &agentItem : this->agentStatesRecord)
    {
        auto &relevantAgentState = agentItem.second;

        smf_move_base_msgs::RelevantAgentState newRelevantAgentState;
        newRelevantAgentState.header = relevantAgentState.header;
        newRelevantAgentState.agent_state = relevantAgentState.agent_state;

        newRelevantAgentState.relevance = relevantAgentState.relevance - double(this->timeDecayFactor * exp(double(this->timeDecayFactor * (ros::Time::now().sec - relevantAgentState.agent_state.header.stamp.now().sec))));

        if (newRelevantAgentState.relevance < 0)
        {
            newRelevantAgentState.relevance = 0;
        }

        agentItem.second = newRelevantAgentState;
    }
}

void SocialCostmap::addNewAgentStates(pedsim_msgs::AgentStates *agentStates)
{

    for (int i = 0; i < agentStates->agent_states.size(); i++)
    {

        smf_move_base_msgs::RelevantAgentState relevantAgentState;

        relevantAgentState.header = agentStates->header;

        relevantAgentState.agent_state = agentStates->agent_states[i];

        relevantAgentState.relevance = 100;

        agentStatesRecord[agentStates->agent_states[i].id] = relevantAgentState;
    }
}

void SocialCostmap::initSocialCostmap()
{
    // social costmap initialization

    // header
    this->socialCostmap.header.stamp = ros::Time::now();
    this->socialCostmap.header.frame_id = this->frameId_;

    // info
    this->socialCostmap.info.resolution = this->resolution_;
    this->socialCostmap.info.width = this->width_;
    this->socialCostmap.info.height = this->height_;
    this->socialCostmap.info.origin = this->origin_;

    int dataArraySize = this->width_ * this->height_;

    this->socialCostmap.data.resize(dataArraySize);

    this->lastUpdateTime_ = ros::Time::now().sec;
}

// ! SETTERS
void SocialCostmap::setDimensions(unsigned int width, unsigned int height)
{
    this->width_ = int(width / this->resolutionFactor);
    this->height_ = int(height / this->resolutionFactor);
}

void SocialCostmap::setOrigin(geometry_msgs::Pose origin)
{
    this->origin_ = origin;
}

void SocialCostmap::setTimeDecayFactor(double timeDecayFactor)
{
    this->timeDecayFactor = timeDecayFactor;
}

void SocialCostmap::setResolution(double resolution)
{
    this->resolution_ = resolution * this->resolutionFactor;
}

void SocialCostmap::setFrameId(std::string frameId)
{
    this->frameId_ = frameId;
}

void SocialCostmap::setResolutionFactor(unsigned int resolutionFactor)
{
    this->resolutionFactor = resolutionFactor;
}


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
    this->socialCostmap.header.frame_id = this->frameId;

    // info
    this->socialCostmap.info.resolution = this->resolution;
    this->socialCostmap.info.width = this->width;
    this->socialCostmap.info.height = this->height;
    this->socialCostmap.info.origin = this->origin;

    int dataArraySize = this->width * this->height;

    this->socialCostmap.data.resize(dataArraySize);

    double mapOriginX = this->origin.position.x + (double(this->width / 2) * this->resolution);

    double mapOriginY = this->origin.position.y + (this->height / 2) * this->resolution;

    for (int j = 0; j < this->height; j++)
    {
        for (int i = 0; i < this->width; i++)
        {
            double wX = mapWx(mapOriginX, this->width, this->resolution, i);
            double wY = mapWy(mapOriginY, this->height, this->resolution, j);

            this->socialCostmap.data[mapIndex(this->width, i, j)] = this->calculateSocialCost(wX, wY);
        }
    }
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

    for (auto &agentItem : this->agentStatesRecord)
    {
        auto &relevantAgentState = agentItem.second;

        smf_move_base_msgs::RelevantAgentState newRelevantAgentState;
        newRelevantAgentState.header = relevantAgentState.header;
        newRelevantAgentState.agent_state = relevantAgentState.agent_state;
        // newRelevantAgentState.relevance = relevantAgentState.relevance - this->timeDecayFactor * exp(ros::Time::now().sec - relevantAgentState.agent_state.header.stamp.now().sec);

        // newRelevantAgentState.relevance = relevantAgentState.relevance - exp(double(this->timeDecayFactor * (ros::Time::now().sec - relevantAgentState.agent_state.header.stamp.now().sec)));

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
    this->socialCostmap.header.frame_id = this->frameId;

    // info
    this->socialCostmap.info.resolution = this->resolution;
    this->socialCostmap.info.width = this->width;
    this->socialCostmap.info.height = this->height;
    this->socialCostmap.info.origin = this->origin;

    int dataArraySize = this->width * this->height;

    this->socialCostmap.data.resize(dataArraySize);

    this->lastUpdateTime = ros::Time::now().sec;
}

// ! SETTERS
void SocialCostmap::setDimensions(unsigned int width, unsigned int height)
{
    this->width = int(width / this->resolutionFactor);
    this->height = int(height / this->resolutionFactor);
}

void SocialCostmap::setOrigin(geometry_msgs::Pose origin)
{
    this->origin = origin;
}

void SocialCostmap::setTimeDecayFactor(double timeDecayFactor)
{
    this->timeDecayFactor = timeDecayFactor;
}

void SocialCostmap::setResolution(double resolution)
{
    this->resolution = resolution * this->resolutionFactor;
}

void SocialCostmap::setFrameId(std::string frameId)
{
    this->frameId = frameId;
}

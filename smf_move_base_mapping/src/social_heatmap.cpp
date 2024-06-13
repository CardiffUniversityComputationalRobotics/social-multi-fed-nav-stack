#include "social_heatmap.hpp"

//! CONSTRUCTOR
SocialHeatmap::SocialHeatmap()
{
}

SocialHeatmap::SocialHeatmap(grid_map::GridMap grid_map)
{
    social_heatmap_ = grid_map;
}

//! FUNCTIONS
void SocialHeatmap::updateSocialHeatmap(grid_map::GridMap grid_map, pedsim_msgs::msg::AgentStates agent_states)
{
    updateAgentStatesRelevance(agent_states);

    addNewAgentStates(agent_states);

    social_heatmap_ = grid_map;

    social_heatmap_.add("social_heatmap");

    grid_map::Matrix &social_heatmap_grid_map = social_heatmap_["social_heatmap"];

    for (auto &agentItem : agent_states_record_)
    {
        auto &relevant_agent_state = agentItem.second;

        grid_map::Position center(relevant_agent_state.agent_state.pose.position.x, relevant_agent_state.agent_state.pose.position.y);

        for (grid_map::CircleIterator iterator(social_heatmap_, center, 2.25);
             !iterator.isPastEnd(); ++iterator)
        {
            try
            {
                grid_map::Position temp_pos;
                social_heatmap_.getPosition(*iterator, temp_pos);

                grid_map::Index index(*iterator);

                double last_val = social_heatmap_grid_map(index(0), index(1));

                if (std::isnan(last_val))
                {
                    last_val = socialComfortCost(temp_pos[0], temp_pos[1], relevant_agent_state);
                }
                else
                {
                    last_val += socialComfortCost(temp_pos[0], temp_pos[1], relevant_agent_state);
                }

                if (last_val > 100)
                {
                    last_val = 100;
                }

                social_heatmap_grid_map(index(0), index(1)) = last_val;
            }
            catch (const std::out_of_range &oor)
            {
                RCLCPP_ERROR(rclcpp::get_logger("SocialHeatmap"), "TRIED TO DEFINE COMFORT OF SOCIAL HEATMAP OUT OF RANGE");
            }
        }
    }
}

void SocialHeatmap::updateAgentStatesRelevance(pedsim_msgs::msg::AgentStates agentStates)
{
    std::vector<int> irrelevant_agents;

    for (auto &agentItem : agent_states_record_)
    {
        if (agentItem.second.relevance <= 0)
        {
            irrelevant_agents.push_back(agentItem.first);
        }
    }

    for (int i = 0; i < irrelevant_agents.size(); i++)
    {
        agent_states_record_.erase(irrelevant_agents[i]);
    }

    for (auto &agentItem : agent_states_record_)
    {
        auto &relevant_agent_state = agentItem.second;

        smf_move_base_msgs::msg::RelevantAgentState new_relevant_agent_state;
        new_relevant_agent_state.header = relevant_agent_state.header;
        new_relevant_agent_state.agent_state = relevant_agent_state.agent_state;
        new_relevant_agent_state.last_time = relevant_agent_state.last_time;

        auto now = rclcpp::Clock().now();
        auto duration = now.seconds() - relevant_agent_state.last_time;

        new_relevant_agent_state.relevance = 100 * exp(double(-time_decay_factor_ * duration / 60));

        if (new_relevant_agent_state.relevance < 0)
        {
            new_relevant_agent_state.relevance = 0;
        }

        agentItem.second = new_relevant_agent_state;
    }
}

void SocialHeatmap::addNewAgentStates(pedsim_msgs::msg::AgentStates agent_state)
{
    for (int i = 0; i < agent_state.agent_states.size(); i++)
    {
        smf_move_base_msgs::msg::RelevantAgentState relevant_agent_state;
        relevant_agent_state.header = agent_state.header;

        relevant_agent_state.last_time = rclcpp::Clock().now().seconds();
        relevant_agent_state.agent_state = agent_state.agent_states[i];
        relevant_agent_state.relevance = 100;

        agent_states_record_[agent_state.agent_states[i].id] = relevant_agent_state;
    }
}

// ! SETTERS

void SocialHeatmap::setTimeDecayFactor(double time_decay_factor)
{
    time_decay_factor_ = time_decay_factor;
}

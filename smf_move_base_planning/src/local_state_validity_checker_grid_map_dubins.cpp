/*! \file state_validity_checker_octomap_fcl_Dubins.cpp
 * \brief State validity checker.
 *
 * \date March 5, 2015
 * \author Juan David Hernandez Vega, juandhv@rice.edu
 *
 * \details Check is a given configuration Dubins is collision-free.
 *  The workspace is represented by an Octomap and collision check is done with FCL.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#include <local_state_validity_checker_grid_map_dubins.h>

LocalGridMapStateValidityCheckerDubins::LocalGridMapStateValidityCheckerDubins(const ob::SpaceInformationPtr &si,
                                                                               const bool opport_collision_check,
                                                                               std::vector<double> planning_bounds_x,
                                                                               std::vector<double> planning_bounds_y, grid_map_msgs::GridMap grid_map)
    : ob::StateValidityChecker(si), local_nh_("~"), robot_base_radius_(0.4)
{

    opport_collision_check_ = opport_collision_check;
    planning_bounds_x_ = planning_bounds_x;
    planning_bounds_y_ = planning_bounds_y;

    local_nh_.param("robot_base_radius", robot_base_radius_, robot_base_radius_);
    local_nh_.param("grid_map_service", grid_map_service_, grid_map_service_);
    local_nh_.param("local_use_social_heatmap", local_use_social_heatmap_, local_use_social_heatmap_);
    local_nh_.param("state_space", state_space_, state_space_);

    if (grid_map::GridMapRosConverter::fromMessage(grid_map, grid_map_))
    {
        ROS_DEBUG("Obtained gridmap successfully");
        grid_map_msgs_ = grid_map;

        grid_map_max_x_ = grid_map_msgs_.info.pose.position.x + (grid_map_msgs_.info.length_x / 2);
        grid_map_min_x_ = grid_map_msgs_.info.pose.position.x - (grid_map_msgs_.info.length_x / 2);

        grid_map_max_y_ = grid_map_msgs_.info.pose.position.y + (grid_map_msgs_.info.length_y / 2);
        grid_map_min_y_ = grid_map_msgs_.info.pose.position.y - (grid_map_msgs_.info.length_y / 2);
    }
    else
    {
        ROS_ERROR("Error reading GridMap");
    }

    try
    {
        full_grid_map_ = grid_map_["full"];
        comfort_grid_map_ = grid_map_["comfort"];
        social_heatmap_grid_map_ = grid_map_["social_heatmap"];
    }
    catch (...)
    {
    }
}

bool LocalGridMapStateValidityCheckerDubins::isValid(const ob::State *state) const
{

    if (state_space_.compare("dubins") == 0)
    {
        const ob::DubinsStateSpace::StateType *state_dubins = state->as<ob::DubinsStateSpace::StateType>();

        if (opport_collision_check_ &&
            (state_dubins->getX() < grid_map_min_x_ || state_dubins->getY() < grid_map_min_y_ ||
             state_dubins->getX() > grid_map_max_x_ || state_dubins->getY() > grid_map_max_y_))
        {
            return true;
        }

        if (state_dubins->getX() < planning_bounds_x_[0] || state_dubins->getY() < planning_bounds_y_[0] ||
            state_dubins->getX() > planning_bounds_x_[1] || state_dubins->getY() > planning_bounds_y_[1])
        {
            return false;
        }

        grid_map::Position query(state_dubins->getX(), state_dubins->getY());

        for (grid_map::CircleIterator iterator(grid_map_, query, robot_base_radius_);
             !iterator.isPastEnd(); ++iterator)
        {
            const grid_map::Index index(*iterator);

            if (full_grid_map_(index(0), index(1)) > 50)
            {
                return false;
            }
        }
    }
    else
    {

        const ob::RealVectorStateSpace::StateType *state_dubins = state->as<ob::RealVectorStateSpace::StateType>();

        if (opport_collision_check_ &&
            (state_dubins->values[0] < grid_map_min_x_ || state_dubins->values[1] < grid_map_min_y_ ||
             state_dubins->values[0] > grid_map_max_x_ || state_dubins->values[1] > grid_map_max_y_))
        {
            return true;
        }

        if (state_dubins->values[0] < planning_bounds_x_[0] || state_dubins->values[1] < planning_bounds_y_[0] ||
            state_dubins->values[0] > planning_bounds_x_[1] || state_dubins->values[1] > planning_bounds_y_[1])
        {
            return false;
        }
        grid_map::Position query(state_dubins->values[0], state_dubins->values[1]);

        for (grid_map::CircleIterator iterator(grid_map_, query, robot_base_radius_);
             !iterator.isPastEnd(); ++iterator)
        {
            const grid_map::Index index(*iterator);

            if (full_grid_map_(index(0), index(1)) > 50)
            {
                return false;
            }
        }
    }

    return true;
}

double LocalGridMapStateValidityCheckerDubins::checkExtendedSocialComfort(const ob::State *state,
                                                                          const ob::SpaceInformationPtr space) const
{

    double state_risk = 0.0;
    grid_map::Index index;

    if (state_space_.compare("dubins") == 0)
    {
        const ob::DubinsStateSpace::StateType *state_dubins = state->as<ob::DubinsStateSpace::StateType>();
        grid_map::Position query(state_dubins->getX(), state_dubins->getY());

        if (grid_map_.getIndex(query, index))
        {
            state_risk = comfort_grid_map_(index(0), index(1));

            if (state_risk < 1 || isnan(state_risk))
            {
                state_risk = 1;
            }

            if (local_use_social_heatmap_)
            {
                double social_heatmap_risk = social_heatmap_grid_map_(index(0), index(1));
                if (!isnan(social_heatmap_risk) && !social_heatmap_risk <= 1)
                {
                    state_risk += social_heatmap_risk / 100;
                }
            }
        }
    }
    else
    {
        const ob::RealVectorStateSpace::StateType *state_dubins = state->as<ob::RealVectorStateSpace::StateType>();
        grid_map::Position query(state_dubins->values[0], state_dubins->values[1]);

        if (grid_map_.getIndex(query, index))
        {
            state_risk = comfort_grid_map_(index(0), index(1));

            if (state_risk < 1 || isnan(state_risk))
            {
                state_risk = 1;
            }

            if (local_use_social_heatmap_)
            {
                double social_heatmap_risk = social_heatmap_grid_map_(index(0), index(1));
                if (!isnan(social_heatmap_risk) && !social_heatmap_risk <= 1)
                {
                    state_risk += social_heatmap_risk / 100;
                }
            }
        }
    }

    return state_risk;
}

bool LocalGridMapStateValidityCheckerDubins::isValidPoint(const ob::State *state) const
{
    // extract the component of the state and cast it to what we expect

    grid_map::Index index;

    if (state_space_.compare("dubins") == 0)
    {
        const ob::DubinsStateSpace::StateType *state_dubins = state->as<ob::DubinsStateSpace::StateType>();
        grid_map::Position query(state_dubins->getX(), state_dubins->getY());

        if (grid_map_.getIndex(query, index))
        {
            if (full_grid_map_(index(0), index(1)) > 50)
            {
                return false;
            }
        }
    }
    else
    {
        const ob::RealVectorStateSpace::StateType *state_dubins = state->as<ob::RealVectorStateSpace::StateType>();
        grid_map::Position query(state_dubins->values[0], state_dubins->values[1]);

        if (grid_map_.getIndex(query, index))
        {
            if (full_grid_map_(index(0), index(1)) > 50)
            {
                return false;
            }
        }
    }

    return true;
}

LocalGridMapStateValidityCheckerDubins::~LocalGridMapStateValidityCheckerDubins()
{
}

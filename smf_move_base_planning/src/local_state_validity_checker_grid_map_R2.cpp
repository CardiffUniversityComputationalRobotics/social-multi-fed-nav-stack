/*! \file state_validity_checker_octomap_fcl_R2.cpp
 * \brief State validity checker.
 *
 * \date March 5, 2015
 * \author Juan David Hernandez Vega, juandhv@rice.edu
 *
 * \details Check is a given configuration R2 is collision-free.
 *  The workspace is represented by an Octomap and collision check is done with FCL.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#include <local_state_validity_checker_grid_map_R2.h>

LocalGridMapStateValidityCheckerR2::LocalGridMapStateValidityCheckerR2(const ob::SpaceInformationPtr &si,
                                                                       const bool opport_collision_check,
                                                                       std::vector<double> planning_bounds_x,
                                                                       std::vector<double> planning_bounds_y)
    : ob::StateValidityChecker(si), local_node_(), robot_base_radius_(0.4)
{
    auto req = std::make_shared<GetGridMap::Request>();

    opport_collision_check_ = opport_collision_check;
    planning_bounds_x_ = planning_bounds_x;
    planning_bounds_y_ = planning_bounds_y;

    local_node_->declare_parameter<std::string>("grid_map_service", "get_grid_map_service");
    local_node_->declare_parameter<double>("robot_base_radius", 0.4);
    local_node_->declare_parameter<bool>("local_use_social_heatmap", false);

    local_node_->get_parameter("grid_map_service", grid_map_service_);
    local_node_->get_parameter("robot_base_radius", robot_base_radius_);
    local_node_->get_parameter("local_use_social_heatmap", local_use_social_heatmap_);

    grid_map_client_ = local_node_->create_client<GetGridMap>(grid_map_service_);

    // ! GRID MAP REQUEST
    RCLCPP_DEBUG(local_node_->get_logger(), "requesting the GridMap to ", grid_map_service_);

    auto result = grid_map_client_->async_send_request(req);

    if (rclcpp::spin_until_future_complete(local_node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        if (grid_map::GridMapRosConverter::fromMessage(result.get()->map, grid_map_))
        {
            RCLCPP_INFO(local_node_->get_logger(), "Obtained gridmap successfully");
            grid_map_msgs_ = result.get()->map;

            grid_map_max_x_ = grid_map_msgs_.info.pose.position.x + (grid_map_msgs_.info.length_x / 2);
            grid_map_min_x_ = grid_map_msgs_.info.pose.position.x - (grid_map_msgs_.info.length_x / 2);

            grid_map_max_y_ = grid_map_msgs_.info.pose.position.y + (grid_map_msgs_.info.length_y / 2);
            grid_map_min_y_ = grid_map_msgs_.info.pose.position.y - (grid_map_msgs_.info.length_y / 2);
        }
    }
    else
    {
        RCLCPP_ERROR(local_node_->get_logger(), "Error reading GridMap");
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

bool LocalGridMapStateValidityCheckerR2::isValid(const ob::State *state) const
{

    if (state_space_.compare("dubins") == 0)
    {
        const ob::DubinsStateSpace::StateType *state_r2 = state->as<ob::DubinsStateSpace::StateType>();

        if (opport_collision_check_ &&
            (state_r2->getX() < grid_map_min_x_ || state_r2->getY() < grid_map_min_y_ ||
             state_r2->getX() > grid_map_max_x_ || state_r2->getY() > grid_map_max_y_))
        {
            return true;
        }

        if (state_r2->getX() < planning_bounds_x_[0] || state_r2->getY() < planning_bounds_y_[0] ||
            state_r2->getX() > planning_bounds_x_[1] || state_r2->getY() > planning_bounds_y_[1])
        {
            return false;
        }

        grid_map::Position query(state_r2->getX(), state_r2->getY());

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

        const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

        if (opport_collision_check_ &&
            (state_r2->values[0] < grid_map_min_x_ || state_r2->values[1] < grid_map_min_y_ ||
             state_r2->values[0] > grid_map_max_x_ || state_r2->values[1] > grid_map_max_y_))
        {
            return true;
        }

        if (state_r2->values[0] < planning_bounds_x_[0] || state_r2->values[1] < planning_bounds_y_[0] ||
            state_r2->values[0] > planning_bounds_x_[1] || state_r2->values[1] > planning_bounds_y_[1])
        {
            return false;
        }
        grid_map::Position query(state_r2->values[0], state_r2->values[1]);

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

double LocalGridMapStateValidityCheckerR2::checkExtendedSocialComfort(const ob::State *state,
                                                                      const ob::SpaceInformationPtr space) const
{

    double state_risk = 0.0;
    grid_map::Index index;

    if (state_space_.compare("dubins") == 0)
    {
        const ob::DubinsStateSpace::StateType *state_r2 = state->as<ob::DubinsStateSpace::StateType>();
        grid_map::Position query(state_r2->getX(), state_r2->getY());

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
        const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();
        grid_map::Position query(state_r2->values[0], state_r2->values[1]);

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

bool LocalGridMapStateValidityCheckerR2::isValidPoint(const ob::State *state) const
{
    // extract the component of the state and cast it to what we expect

    grid_map::Index index;

    if (state_space_.compare("dubins") == 0)
    {
        const ob::DubinsStateSpace::StateType *state_r2 = state->as<ob::DubinsStateSpace::StateType>();
        grid_map::Position query(state_r2->getX(), state_r2->getY());

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
        const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();
        grid_map::Position query(state_r2->values[0], state_r2->values[1]);

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

LocalGridMapStateValidityCheckerR2::~LocalGridMapStateValidityCheckerR2()
{
}

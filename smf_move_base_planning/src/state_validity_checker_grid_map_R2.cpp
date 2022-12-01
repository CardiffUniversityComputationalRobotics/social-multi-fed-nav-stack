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

#include <state_validity_checker_grid_map_R2.h>

OmFclStateValidityCheckerR2::OmFclStateValidityCheckerR2(const ob::SpaceInformationPtr &si,
                                                         const bool opport_collision_check,
                                                         std::vector<double> planning_bounds_x,
                                                         std::vector<double> planning_bounds_y)
    : ob::StateValidityChecker(si), local_nh_("~"), robot_base_radius_(0.4), robot_base_height_(2.0)
{
    GetOctomap::Request req;
    GetOctomap::Response resp;

    opport_collision_check_ = opport_collision_check;
    planning_bounds_x_ = planning_bounds_x;
    planning_bounds_y_ = planning_bounds_y;

    local_nh_.param("robot_base_radius", robot_base_radius_, robot_base_radius_);
    local_nh_.param("robot_base_height", robot_base_height_, robot_base_height_);
    local_nh_.param("optimization_objective", optimization_objective, optimization_objective);
}

bool OmFclStateValidityCheckerR2::isValid(const ob::State *state) const
{
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    // ompl::tools::Profiler::Begin("collision");

    // extract the component of the state and cast it to what we expect

    if (opport_collision_check_ &&
        (state_r2->values[0] < octree_min_x_ || state_r2->values[1] < octree_min_y_ ||
         state_r2->values[0] > octree_max_x_ || state_r2->values[1] > octree_max_y_))
    {
        // ompl::tools::Profiler::End("collision");
        return true;
    }

    if (state_r2->values[0] < planning_bounds_x_[0] || state_r2->values[1] < planning_bounds_y_[0] ||
        state_r2->values[0] > planning_bounds_x_[1] || state_r2->values[1] > planning_bounds_y_[1])
    {
        // ompl::tools::Profiler::End("collision");
        return false;
    }

    return true;
}

double OmFclStateValidityCheckerR2::checkSocialCostmap(const ob::State *state,
                                                       const ob::SpaceInformationPtr space) const
{
    // ROS_INFO_STREAM("Running social costmap cost objective");

    double state_risk = 1.0;

    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    if (state_risk < 1)
    {
        state_risk = 1.0;
        return state_risk;
    }

    return state_risk;
}

bool OmFclStateValidityCheckerR2::isValidPoint(const ob::State *state) const
{
    // extract the component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    // query.x() = state_r2->values[0];
    // query.y() = state_r2->values[1];
    // query.z() = 0.0;

    // result = octree_->search(query);

    // if (result == NULL)
    // {
    //     return false;
    // }
    // else
    // {
    //     node_occupancy = result->getOccupancy();
    //     if (node_occupancy <= 0.4)
    //         return true;
    // }
    return false;
}

OmFclStateValidityCheckerR2::~OmFclStateValidityCheckerR2()
{
}

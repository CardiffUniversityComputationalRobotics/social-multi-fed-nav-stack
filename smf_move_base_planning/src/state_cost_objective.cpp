/*! \file state_cost_objective.cpp
 * \brief State cost objective.
 *
 * \date Mar 31, 2015
 * \author Juan David Hernandez Vega, juandhv@rice.edu
 *
 * \details State cost objective. Define different state cost objectives.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#include <state_cost_objective.h>

// !SOCIAL COMFORT MODEL
ob::OptimizationObjectivePtr getSocialComfortObjective(const ob::SpaceInformationPtr &si,
                                                       bool motion_cost_interpolation, const ob::StateSpacePtr &space, const ob::PlannerPtr &planner,
                                                       const std::vector<const ob::State *> &start_states)
{
    return ob::OptimizationObjectivePtr(new SocialComfortObjective(si, motion_cost_interpolation, space, planner, start_states));
}

// !SOCIAL COSTMAP
ob::OptimizationObjectivePtr getSocialHeatmapObjective(const ob::SpaceInformationPtr &si,
                                                       bool motion_cost_interpolation)
{
    return ob::OptimizationObjectivePtr(new SocialHeatmapObjective(si, motion_cost_interpolation));
}

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr &si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

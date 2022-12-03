/*! \file state_cost_objective.hpp
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

#ifndef OMPL_CONTRIB_STATE_COST_OBJECTIVES_
#define OMPL_CONTRIB_STATE_COST_OBJECTIVES_

// OMPL
#include <ompl/config.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
#include <sampler/PathLengthDirectInfSamplerMod.h>

#include <state_validity_checker_grid_map_R2.h>
#include <local_state_validity_checker_grid_map_R2.h>

namespace ob = ompl::base;

/*
 * Class to manage integral cost objective based on extended social comfort function.
 */
class SocialComfortObjective : public ob::StateCostIntegralObjective
{
private:
public:
    ob::StateSpacePtr space_;
    ob::PlannerPtr planner_;
    std::vector<ob::State *> start_states_;

    SocialComfortObjective(const ob::SpaceInformationPtr &si,
                           bool enableMotionCostInterpolation, const ob::StateSpacePtr &space, const ob::PlannerPtr &planner,
                           const std::vector<const ob::State *> &start_states)
        : ob::StateCostIntegralObjective(si, enableMotionCostInterpolation)
    {
        space_ = si_->getStateSpace();
        planner_ = planner;

        for (int i = 0; i < start_states.size(); i++)
        {
            ob::State *s = space_->allocState();
            space_->copyState(s, start_states[i]);
            start_states_.push_back(s);
        }
    }

    // in case of modifying the cost calculation function here it is where it should be done

    ob::Cost stateCost(const ob::State *s) const
    {
        // ROS_INFO_STREAM("Running social comfort model");
        std::shared_ptr<LocalGridMapStateValidityCheckerR2> state_vality_checker =
            std::static_pointer_cast<LocalGridMapStateValidityCheckerR2>(si_->getStateValidityChecker());
        return ob::Cost(state_vality_checker->checkExtendedSocialComfort(s, si_));
    }

    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const
    {
        if (interpolateMotionCost_)
        {
            ob::Cost totalCost = this->identityCost();

            int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
            // nd = int(nd/10);

            ob::State *test1 = si_->cloneState(s1);
            ob::Cost prevStateCost = this->stateCost(test1);
            if (nd > 1)
            {
                ob::State *test2 = si_->allocState();
                for (int j = 1; j < nd; ++j)
                {
                    si_->getStateSpace()->interpolate(s1, s2, (double)j / (double)nd, test2);
                    ob::Cost nextStateCost = this->stateCost(test2);
                    totalCost = ob::Cost(
                        totalCost.value() +
                        this->trapezoid(prevStateCost, nextStateCost, si_->distance(test1, test2)).value());
                    std::swap(test1, test2);
                    prevStateCost = nextStateCost;
                }
                si_->freeState(test2);
            }

            // Lastly, add s2
            totalCost = ob::Cost(
                totalCost.value() +
                this->trapezoid(prevStateCost, this->stateCost(s2), si_->distance(test1, s2)).value());

            si_->freeState(test1);

            // ROS_INFO_STREAM("Total cost: " << totalCost);

            return totalCost;
        }
        else
        {
            // ROS_INFO_STREAM("Trapezoid cost: " << this->trapezoid(this->stateCost(s1), this->stateCost(s2),
            //                                                       si_->distance(s1, s2)));
            return this->trapezoid(this->stateCost(s1), this->stateCost(s2), si_->distance(s1, s2));
        }
    }

    /** \brief Allocate a state sampler for the path-length objective (i.e., direct ellipsoidal sampling). */
    ob::InformedSamplerPtr allocInformedStateSampler(const ob::ProblemDefinitionPtr &probDefn,
                                                     unsigned int maxNumberCalls) const
    {
        // Make the direct path-length informed sampler and return. If OMPL was compiled with Eigen, a direct
        // version is available, if not a rejection-based technique can be used
        return std::make_shared<ob::PathLengthDirectInfSamplerMod>(probDefn, maxNumberCalls, space_, planner_, start_states_);
    }
};

/*
 * Class to manage integral cost objective based on social costmap.
 */
class SocialHeatmapObjective : public ob::StateCostIntegralObjective
{
private:
public:
    SocialHeatmapObjective(const ob::SpaceInformationPtr &si, bool enableMotionCostInterpolation)
        : ob::StateCostIntegralObjective(si, enableMotionCostInterpolation)
    {
    }

    // in case of modifying the cost calculation function here it is where it should be done

    ob::Cost stateCost(const ob::State *s) const
    {
        // ROS_INFO_STREAM("Running social costmap model");
        std::shared_ptr<GridMapStateValidityCheckerR2> state_vality_checker =
            std::static_pointer_cast<GridMapStateValidityCheckerR2>(si_->getStateValidityChecker());
        return ob::Cost(state_vality_checker->checkSocialHeatmap(s, si_));
    }

    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const
    {
        if (interpolateMotionCost_)
        {
            ob::Cost totalCost = this->identityCost();

            int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
            // nd = int(nd/10);

            ob::State *test1 = si_->cloneState(s1);
            ob::Cost prevStateCost = this->stateCost(test1);
            if (nd > 1)
            {
                ob::State *test2 = si_->allocState();
                for (int j = 1; j < nd; ++j)
                {
                    si_->getStateSpace()->interpolate(s1, s2, (double)j / (double)nd, test2);
                    ob::Cost nextStateCost = this->stateCost(test2);
                    totalCost = ob::Cost(
                        totalCost.value() +
                        this->trapezoid(prevStateCost, nextStateCost, si_->distance(test1, test2)).value());
                    std::swap(test1, test2);
                    prevStateCost = nextStateCost;
                }
                si_->freeState(test2);
            }

            // Lastly, add s2
            totalCost = ob::Cost(
                totalCost.value() +
                this->trapezoid(prevStateCost, this->stateCost(s2), si_->distance(test1, s2)).value());

            si_->freeState(test1);

            // ROS_INFO_STREAM("Total cost: " << totalCost);

            return totalCost;
        }
        else
        {
            // ROS_INFO_STREAM("Trapezoid cost: " << this->trapezoid(this->stateCost(s1), this->stateCost(s2),
            //                                                       si_->distance(s1, s2)));
            return this->trapezoid(this->stateCost(s1), this->stateCost(s2), si_->distance(s1, s2));
        }
    }
};

/** Return an optimization objective which attempts to steer the robot
    away from social agents according to an extended social comfort model */
ob::OptimizationObjectivePtr getSocialComfortObjective(const ob::SpaceInformationPtr &si,
                                                       bool motion_cost_interpolation, const ob::StateSpacePtr &space, const ob::PlannerPtr &planner,
                                                       const std::vector<const ob::State *> &start_states);

/** Return an optimization objective which attempts to steer the robot
    away from social agents according to a costmap provided by a world modeling module */
ob::OptimizationObjectivePtr getSocialHeatmapObjective(const ob::SpaceInformationPtr &si,
                                                       bool motion_cost_interpolation);

/** Returns a structure representing the optimization objective to use
    for optimal motion planning. This method returns an objective
    which attempts to minimize the length in configuration space of
    computed paths. */
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr &si);

#endif

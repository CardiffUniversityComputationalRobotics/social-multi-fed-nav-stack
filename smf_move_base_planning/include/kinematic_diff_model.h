/*
 * motion_model_auv.hpp
 *
 *  Created on: December 1, 2014
 *      Author: juandhv (Juan David Hernandez Vega, juandhv@eia.udg.edu)
 *
 *  Differential models of for Unicycle.
 *  Kinematic model.
 */

// Standard libraries
#include <cstdlib>
#include <cmath>
#include <string>

// OMPL
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ODESolver.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>

// Boost
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>

// Eigen
#include <Eigen/Dense>

// Standard namespace
using namespace std;

// OMPL namespaces
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

//!  KinematicDiffModel class.
/*!
 * AUV kinematic model.
 */
class KinematicDiffModel
{
public:
    //! Constructor.
    KinematicDiffModel(const ob::StateSpace *space);

    /// implement the function describing the robot motion: qdot = f(q, u)
    void operator()(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const;

    /// implement y(n+1) = y(n) + d
    void update(ob::State *state, const std::valarray<double> &dstate) const;

private:
    const ob::StateSpace *space_;
};

//!  EulerIntegrator class.
/*!
 * Simple integrator: Euclidean method.
 */
template <typename F>
class EulerIntegrator
{
public:
    EulerIntegrator(const ob::StateSpace *space, double timeStep);

    void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const;

    double getTimeStep(void) const;

    void setTimeStep(double timeStep);

private:
    const ob::StateSpace *space_;
    double timeStep_;
    F ode_;
};

/// @cond IGNORE

//!  KinDiffControlSpace class.
/*!
 * Control space for an AUV kinematic model.
 */
class KinDiffControlSpace : public oc::RealVectorControlSpace
{
public:
    KinDiffControlSpace(const ob::StateSpacePtr &stateSpace);
};

class KinDiffStatePropagator : public oc::StatePropagator
{
public:
    KinDiffStatePropagator(const oc::SpaceInformationPtr &si);

    virtual void propagate(const ob::State *state, const oc::Control *control, const double duration, ob::State *result) const;

    void setIntegrationTimeStep(double timeStep);

    double getIntegrationTimeStep(void) const;

    EulerIntegrator<KinematicDiffModel> integrator_;
};

/// @endcond
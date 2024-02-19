/*
 * motion_model_auv.cpp
 *
 *  Created on: December 1, 2014
 *      Author: juandhv (Juan David Hernandez Vega, juandhv@eia.udg.edu)
 *
 *  Differential models of Unicycle.
 *  Kinematic model.
 */

// motion models of auv
#include "kinematic_diff_model.h"

//! Constructor.
KinematicDiffModel::KinematicDiffModel(const ob::StateSpace *space) : space_(space)
{
}

/// implement the function describing the robot motion: qdot = f(q, u)
void KinematicDiffModel::operator()(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();

    dstate.resize(3);
    dstate[0] = u[0] * cos(theta);
    dstate[1] = u[0] * sin(theta);
    dstate[2] = u[1];
}

/// implement y(n+1) = y(n) + d
void KinematicDiffModel::update(ob::State *state, const std::valarray<double> &dstate) const
{
    ob::SE2StateSpace::StateType &s = *state->as<ob::SE2StateSpace::StateType>();
    s.setX(s.getX() + dstate[0]);
    s.setY(s.getY() + dstate[1]);
    s.setYaw(s.getYaw() + dstate[2]);
    space_->enforceBounds(state);
}

//! Constructor for Euler integrator.
template <typename F>
EulerIntegrator<F>::EulerIntegrator(const ob::StateSpace *space, double timeStep) : space_(space), timeStep_(timeStep), ode_(space)
{
}

template <typename F>
void EulerIntegrator<F>::propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
{
    double t = timeStep_;
    std::valarray<double> dstate;
    space_->copyState(result, start);
    while (t < duration + std::numeric_limits<double>::epsilon())
    {
        ode_(result, control, dstate);
        ode_.update(result, timeStep_ * dstate);
        t += timeStep_;
    }
}

template <typename F>
double EulerIntegrator<F>::getTimeStep(void) const
{
    return timeStep_;
}

template <typename F>
void EulerIntegrator<F>::setTimeStep(double timeStep)
{
    timeStep_ = timeStep;
}

/// @cond IGNORE

KinDiffControlSpace::KinDiffControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
{
}

KinDiffStatePropagator::KinDiffStatePropagator(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si),
                                                                                    integrator_(si->getStateSpace().get(), 0.0)
{
}

void KinDiffStatePropagator::propagate(const ob::State *state, const oc::Control *control, const double duration, ob::State *result) const
{
    integrator_.propagate(state, control, duration, result);
}

void KinDiffStatePropagator::setIntegrationTimeStep(double timeStep)
{
    integrator_.setTimeStep(timeStep);
}

double KinDiffStatePropagator::getIntegrationTimeStep(void) const
{
    return integrator_.getTimeStep();
}

/// @endcond
#include <kinematic_diff_model.h>

KinematicDiffModel::KinematicDiffModel(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
{
    space_ = si->getStateSpace();
    timeStep_ = 0.01;
}

void KinematicDiffModel::propagate(const ob::State *state, const oc::Control *control, const double duration, ob::State *result) const
{
    EulerIntegration(state, control, duration, result);
}

// Explicit Euler Method for numerical integration.
void KinematicDiffModel::EulerIntegration(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
{
    double t = timeStep_;
    std::valarray<double> dstate;
    space_->copyState(result, start);
    while (t < duration + std::numeric_limits<double>::epsilon())
    {
        ode(result, control, dstate);
        update(result, timeStep_ * dstate);
        t += timeStep_;
    }
    if (t + std::numeric_limits<double>::epsilon() > duration)
    {
        ode(result, control, dstate);
        update(result, (t - duration) * dstate);
    }
}

void KinematicDiffModel::ode(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();

    dstate.resize(3);
    dstate[0] = u[0] * cos(theta);
    dstate[1] = u[0] * sin(theta);
    dstate[2] = u[0] * tan(u[1]);
}

void KinematicDiffModel::update(ob::State *state, const std::valarray<double> &dstate) const
{
    ob::SE2StateSpace::StateType &s = *state->as<ob::SE2StateSpace::StateType>();
    s.setX(s.getX() + dstate[0]);
    s.setY(s.getY() + dstate[1]);
    s.setYaw(s.getYaw() + dstate[2]);
    space_->enforceBounds(state);
}

// This is a callback method invoked after numerical integration.
void KinematicDiffPostIntegration(const ob::State * /*state*/, const oc::Control * /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
}

// Definition of the ODE for the kinematic car.
// This method is analogous to the above KinematicDiffModel::ode function.
void KinematicCarODE(const oc::ODESolver::StateType &q, const oc::Control *control, oc::ODESolver::StateType &qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];

    // Zero out qdot
    qdot.resize(q.size(), 0);

    qdot[0] = u[0] * cos(theta);
    qdot[1] = u[0] * sin(theta);
    qdot[2] = u[0] * tan(u[1]);
}
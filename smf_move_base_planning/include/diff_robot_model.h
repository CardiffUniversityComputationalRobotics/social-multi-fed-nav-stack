#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>

namespace ob = ompl::base;
namespace oc = ompl::control;

// Kinematic car model object definition.  This class does NOT use ODESolver to propagate the system.
class KinematicDiffModel : public oc::StatePropagator
{
public:
    KinematicDiffModel(const oc::SpaceInformationPtr &si);

    void propagate(const ob::State *state, const oc::Control *control, const double duration, ob::State *result) const override;

protected:
    // Explicit Euler Method for numerical integration.
    void EulerIntegration(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const;

    void ode(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const;

    void update(ob::State *state, const std::valarray<double> &dstate) const;

    ob::StateSpacePtr space_;
    double timeStep_;
};

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

// This is a callback method invoked after numerical integration.
void KinematicDiffPostIntegration(const ob::State * /*state*/, const oc::Control * /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
}

class DemoControlSpace : public oc::RealVectorControlSpace
{
public:
    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
    {
    }
};

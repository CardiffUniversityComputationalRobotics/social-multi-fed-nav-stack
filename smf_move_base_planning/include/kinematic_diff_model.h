

#ifndef OMPL_CONTRIB_KINEMATIC_DIFF_MODEL_
#define OMPL_CONTRIB_KINEMATIC_DIFF_MODEL_

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

class DemoControlSpace : public oc::RealVectorControlSpace
{
public:
    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
    {
    }
};

void KinematicDiffODE(const oc::ODESolver::StateType &q, const oc::Control *control, oc::ODESolver::StateType &qdot);
void KinematicDiffPostIntegration(const ob::State * /*state*/, const oc::Control * /*control*/, const double /*duration*/, ob::State *result);

#endif
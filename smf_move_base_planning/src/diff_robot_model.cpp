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
    KinematicDiffModel(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
    {
        space_ = si->getStateSpace();
        timeStep_ = 0.01;
    }

    void propagate(const ob::State *state, const oc::Control *control, const double duration, ob::State *result) const override
    {
        EulerIntegration(state, control, duration, result);
    }

protected:
    // Explicit Euler Method for numerical integration.
    void EulerIntegration(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
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

    void ode(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
    {
        const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
        const double theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();

        dstate.resize(3);
        dstate[0] = u[0] * cos(theta);
        dstate[1] = u[0] * sin(theta);
        dstate[2] = u[0] * tan(u[1]);
    }

    void update(ob::State *state, const std::valarray<double> &dstate) const
    {
        ob::SE2StateSpace::StateType &s = *state->as<ob::SE2StateSpace::StateType>();
        s.setX(s.getX() + dstate[0]);
        s.setY(s.getY() + dstate[1]);
        s.setYaw(s.getYaw() + dstate[2]);
        space_->enforceBounds(state);
    }

    ob::StateSpacePtr space_;
    double timeStep_;
};

// Definition of the ODE for the kinematic car.
// This method is analogous to the above KinematicDiffModel::ode function.
void KinematicCarODE(const oc::ODESolver::StateType &q, const oc::Control *control, oc::ODESolver::StateType &qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];
    double carLength = 0.2;

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

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (const void *)rot != (const void *)pos;
}

class DemoControlSpace : public oc::RealVectorControlSpace
{
public:
    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
    {
    }
};

// void planWithSimpleSetup()
// {
//     auto space(std::make_shared<ob::SE2StateSpace>());

//     ob::RealVectorBounds bounds(2);
//     bounds.setLow(-1);
//     bounds.setHigh(1);

//     space->setBounds(bounds);

//     // create a control space
//     auto cspace(std::make_shared<DemoControlSpace>(space));

//     // set the bounds for the control space
//     ob::RealVectorBounds cbounds(2);
//     cbounds.setLow(-0.3);
//     cbounds.setHigh(0.3);

//     cspace->setBounds(cbounds);

//     // define a simple setup class
//     oc::SimpleSetup ss(cspace);

//     // set state validity checking for this space
//     oc::SpaceInformation *si = ss.getSpaceInformation().get();
//     ss.setStateValidityChecker(
//         [si](const ob::State *state)
//         { return isStateValid(si, state); });

//     // Setting the propagation routine for this space:
//     // KinematicDiffModel does NOT use ODESolver
//     // ss.setStatePropagator(std::make_shared<KinematicDiffModel>(ss.getSpaceInformation()));

//     // Use the ODESolver to propagate the system.  Call KinematicDiffPostIntegration
//     // when integration has finished to normalize the orientation values.
//     auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &KinematicCarODE));
//     ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicDiffPostIntegration));

//     ob::ScopedState<ob::SE2StateSpace> start(space);
//     start->setX(-0.5);
//     start->setY(0.0);
//     start->setYaw(0.0);

//     ob::ScopedState<ob::SE2StateSpace> goal(space);
//     goal->setX(0.0);
//     goal->setY(0.5);
//     goal->setYaw(0.0);

//     ss.setStartAndGoalStates(start, goal, 0.05);

//     ss.setup();

//     ob::PlannerStatus solved = ss.solve(10.0);

//     if (solved)
//     {
//         std::cout << "Found solution:" << std::endl;

//         ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
//     }
//     else
//         std::cout << "No solution found" << std::endl;
// }

// int main(int /*argc*/, char ** /*argv*/)
// {
//     std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

//     planWithSimpleSetup();

//     return 0;
// }
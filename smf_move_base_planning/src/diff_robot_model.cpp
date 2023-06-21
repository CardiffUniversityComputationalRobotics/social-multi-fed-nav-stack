#include <diff_robot_model.h>

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
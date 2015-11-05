#include "ompl_planner_base_ode.h"

class DemoControlSpace : public oc::RealVectorControlSpace
{
public:

    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
    {
    }
};

KinematicCarModel::KinematicCarModel(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
{
	space_     = si->getStateSpace();
	carLength_ = 0.2;
	timeStep_  = 0.01;
}

void KinematicCarModel::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const
{
	EulerIntegration(state, control, duration, result);
}

void KinematicCarModel::EulerIntegration(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
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

void KinematicCarModel::ode(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
{
	const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
	const double theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();

	dstate.resize(3);
	dstate[0] = u[0] * cos(theta);
	dstate[1] = u[0] * sin(theta);
	dstate[2] = u[0] * tan(u[1]) / carLength_;
}

void KinematicCarModel::update(ob::State *state, const std::valarray<double> &dstate) const
{
	ob::SE2StateSpace::StateType &s = *state->as<ob::SE2StateSpace::StateType>();
	s.setX(s.getX() + dstate[0]);
	s.setY(s.getY() + dstate[1]);
	s.setYaw(s.getYaw() + dstate[2]);
	space_->enforceBounds(state);
}


void KinematicCarODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];
    double carLength = 0.2;

    // Zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = u[0] * cos(theta);
    qdot[1] = u[0] * sin(theta);
    qdot[2] = u[0] * tan(u[1]) / carLength;
}

void KinematicCarPostIntegration (const oc::Control* /*control*/, ob::State* result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds (result->as<ob::SE2StateSpace::StateType>());
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    /// cast the abstract state type to the type we expect
    const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();

    /// extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    /// extract the second component of the state and cast it to what we expect
    const ob::SO2StateSpace::StateType *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    /// check validity of state defined by pos & rot


    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (void*)rot != (void*)pos;
}


void planWithSimpleSetup(double start_x, double start_y, double start_theta,
		double goal_x, double goal_y, double goal_theta)
{
    /// construct the state space we are planning in
    ob::StateSpacePtr space(new ob::SE2StateSpace());

    /// set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // create a control space
    oc::ControlSpacePtr cspace(new DemoControlSpace(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);

    cspace->as<DemoControlSpace>()->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1));

    // Setting the propagation routine for this space:
    // KinematicCarModel does NOT use ODESolver
    //ss.setStatePropagator(oc::StatePropagatorPtr(new KinematicCarModel(ss.getSpaceInformation())));

    // Use the ODESolver to propagate the system.  Call KinematicCarPostIntegration
    // when integration has finished to normalize the orientation values.
//    oc::ODEBasicSolver<> odeSolver(ss.getSpaceInformation(), &KinematicCarODE);
//    ss.setStatePropagator(odeSolver.getStatePropagator(&KinematicCarPostIntegration));
    oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<> (ss.getSpaceInformation(), &KinematicCarODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver/*, &KinematicCarPostIntegration*/));

    /// create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(start_x);
    start->setY(start_y);
    start->setYaw(start_theta);

    /// create a  goal state; use the hard way to set the elements
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(goal_x);
    goal->setY(goal_y);
    goal->setYaw(goal_theta);

    /// set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05);

    /// we want to have a reasonable value for the propagation step size
    ss.setup();

    /// attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        /// print the path to screen

        ss.getSolutionPath().asGeometric().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

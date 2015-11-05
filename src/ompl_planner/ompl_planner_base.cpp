#include "ompl_planner_base.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

static carmen_map_t g_map;

cell_coords_t
to_map_pose(const carmen_map_config_t *map_config, double x, double y)
{
	cell_coords_t p;

	p.x = (int) ((x - map_config->x_origin) / map_config->resolution);
	p.y = (int) ((y - map_config->y_origin) / map_config->resolution);

	return p;
}

bool isMapPositionValid(const carmen_map_t &map, int x, int y)
{
	return x >= 0 && x < map.config.x_size && y >= 0 && y < map.config.y_size;
}

bool isStateValid(const ob::SpaceInformation *si, const ob::State *state)
{
	bool isValid = false;
	const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
	double x=s->getX(), y=s->getY();

	isValid = si->satisfiesBounds(s);
	if (isValid)
	{
		cell_coords_t map_coord_pose = to_map_pose(&g_map.config, x, y);
		isValid = isMapPositionValid(g_map, map_coord_pose.x, map_coord_pose.y);
		if (isValid)
		{
			isValid = g_map.complete_map[map_coord_pose.x * g_map.config.y_size + map_coord_pose.y] <= 0.1;
		}
	}

	return isValid;
}

void planTrajectory(const carmen_map_t &map, ob::StateSpacePtr space, const ob::ScopedState<> &start, const ob::ScopedState<> &goal)
{
	g_map = map;

	ob::RealVectorBounds bounds(2);

	bounds.setLow(0, map.config.x_origin);
	bounds.setHigh(0, map.config.x_origin+(double)map.config.x_size/map.config.resolution);

	bounds.setLow(1, map.config.y_origin);
	bounds.setHigh(1, map.config.y_origin+(double)map.config.y_size/map.config.resolution);

	space->as<ob::SE2StateSpace>()->setBounds(bounds);

	// define a simple setup class
	og::SimpleSetup ss(space);

	// set state validity checking for this space
	ob::SpaceInformationPtr si(ss.getSpaceInformation());
	ss.setStateValidityChecker(boost::bind(&isStateValid, si.get(), _1));

	ss.setStartAndGoalStates(start, goal);

	// this call is optional, but we put it in to get more output information
	ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
	ss.setup();
	ss.print();

	// attempt to solve the problem within 1 second of planning time
	ob::PlannerStatus solved = ss.solve(1.0);

	if (solved)
	{
		std::vector<double> reals;

		std::cout << "Found solution:" << std::endl;
		// We can't use regular simplify because it also tries to use spline interpolation,
		// which doesn't work for Dubins curves.
		//ss.simplifySolution();
		og::PathGeometric path = ss.getSolutionPath();
		og::PathSimplifierPtr ps = ss.getPathSimplifier();
		ps->reduceVertices(path);
		ps->collapseCloseVertices(path);
		path.interpolate(1000);
		for (unsigned int i=0; i < path.getStateCount(); ++i)
		{
			reals = ob::ScopedState<>(space, path.getState(i)).reals();
			std::cout << "path " << reals[0] <<' '<< reals[1] << ' ' << reals[2] << std::endl;
		}
	}
	else
		std::cout << "No solution found" << std::endl;
}

void printTrajectory(ob::StateSpacePtr space, const ob::ScopedState<ob::SE2StateSpace> &goal)
{
	const unsigned int num_pts = 50;
	ob::ScopedState<> from(space), to(space), s(space);
	std::vector<double> reals;

	from[0] = from[1] = from[2] = 0.;

	to[0] = goal->getX();
	to[1] = goal->getY();
	to[2] = goal->getYaw();

	std::cout << "distance: " << space->distance(from(), to()) << "\npath:\n";
	for (unsigned int i=0; i<=num_pts; ++i)
	{
		space->interpolate(from(), to(), (double)i/num_pts, s());
		reals = s.reals();
		std::cout << "path " << reals[0] << ' ' << reals[1] << ' ' << reals[2] << ' ' << std::endl;
	}
}


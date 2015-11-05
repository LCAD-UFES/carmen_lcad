#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/behavior_selector_interface.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "ompl_planner_base.h"

static int necessary_maps_available = 0;
static carmen_point_t robot_pose;
static carmen_point_t goal_pose;
static carmen_map_t carmen_map;
static ob::StateSpacePtr robot_space;

static void
robot_handler(carmen_localize_globalpos_message *message)
{
	if (!necessary_maps_available)
		return;

	robot_pose = message->globalpos;
}

static void
robot_ackerman_handler(carmen_localize_ackerman_globalpos_message *message)
{
	if (!necessary_maps_available)
		return;

	robot_pose = message->globalpos;
}

static void
map_handler(carmen_map_server_offline_map_message *message)
{
	// **************************************************
	// TODO: dessa forma so iremos copiar o primeiro mapa
	// **************************************************

	if (!necessary_maps_available)
	{
		carmen_map_server_copy_offline_map_from_message(&carmen_map, message);
		necessary_maps_available = 1;
		carmen_warn("new map\n");
	}
}

static void
goal_handler(carmen_navigator_set_goal_message *message)
{
	printf("==>>> RECEIVED GOAL\n");

	if (message->x < 0 && message->y < 0)
	{
		return;
	}

	goal_pose.x		= message->x;
	goal_pose.y		= message->y;
	goal_pose.theta 	= 0.0;

	carmen_warn("%lf, %lf\n", goal_pose.x, goal_pose.y);
}

static void
ackerman_goal_handler(carmen_navigator_ackerman_set_goal_message *message)
{
	printf("==>>> RECEIVED ACKERMAN GOAL\n");

	if (message->x < 0 && message->y < 0)
	{
		return;
	}

	goal_pose.x		= message->x;
	goal_pose.y		= message->y;
	goal_pose.theta 	= 0.0;

	carmen_warn("%lf, %lf\n", goal_pose.x, goal_pose.y);
}

static void
publish_status()
{
	if (goal_pose.x < 0 && goal_pose.y < 0)
	{
		return;
	}

	printf("goal %lf %lf %lf\n", goal_pose.x, goal_pose.y, goal_pose.theta);

	ob::ScopedState<ob::SE2StateSpace> start(robot_space);
	start->setX(robot_pose.x);
	start->setY(robot_pose.y);
	start->setYaw(robot_pose.theta);

	ob::ScopedState<ob::SE2StateSpace> goal(robot_space);
	goal->setX(goal_pose.x);
	goal->setY(goal_pose.y);
	goal->setYaw(goal_pose.theta);

	planTrajectory(carmen_map, robot_space, start, goal);
	printTrajectory(robot_space, goal);

	//TODO publicar para o motion planning
}

static void
publish_heartbeats()
{
	carmen_publish_heartbeat((char *)"ompl_planner");
}

void
behavior_selector_goallist_handler(carmen_behavior_selector_goal_list_message *msg __attribute__ ((unused)))
{
	/**
	 * Recebe continuamente a lista de goals
	 */
}

void
add_goal_handler(carmen_behavior_selector_add_goal_message *msg)
{
	/**
	 * Recebe o goal que o usuario selecionou
	 */

	goal_pose.x = msg->goal.x;
	goal_pose.y = msg->goal.y;
	goal_pose.theta = msg->goal.theta;
}

static void
runWithCarmen(int argc, char** argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	carmen_localize_subscribe_globalpos_message(NULL,
			(carmen_handler_t) robot_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_globalpos_message(NULL,
			(carmen_handler_t) robot_ackerman_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_offline_map(NULL,
			(carmen_handler_t) map_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *)CARMEN_NAVIGATOR_SET_GOAL_NAME,
			(char *)CARMEN_NAVIGATOR_SET_GOAL_FMT,
			NULL, sizeof(carmen_navigator_set_goal_message),
			(carmen_handler_t)goal_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME,
			(char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_FMT,
			NULL, sizeof(carmen_navigator_ackerman_set_goal_message),
			(carmen_handler_t)ackerman_goal_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *)CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_NAME,
			(char *)CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_FMT,
			NULL, sizeof(carmen_behavior_selector_add_goal_message),
			(carmen_handler_t)add_goal_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_goal_list_message(NULL, (carmen_handler_t) behavior_selector_goallist_handler, CARMEN_SUBSCRIBE_ALL);

	carmen_ipc_addPeriodicTimer(1.0, (TIMER_HANDLER_TYPE) publish_heartbeats, NULL);
	carmen_ipc_addPeriodicTimer(1.0, (TIMER_HANDLER_TYPE) publish_status, NULL);
	carmen_ipc_dispatch();
}

int main(int argc, char* argv[])
{
	goal_pose.x = -1.0;
	goal_pose.y = -1.0;

	try
	{
		po::options_description desc("Options");
		desc.add_options()
        		    ("help", "show help message")
        		    ("dubins", "use Dubins state space")
        		    ("dubinssym", "use symmetrized Dubins state space")
        		    ("reedsshepp", "use Reeds-Shepp state space (default)")
        		    ;

		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, desc,
				po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
		po::notify(vm);

		if (vm.count("help") || argc==1)
		{
			std::cout << desc << "\n";
			return 1;
		}

		if (vm.count("dubins"))
			robot_space = ob::StateSpacePtr(new ob::DubinsStateSpace);
		else if (vm.count("dubinssym"))
			robot_space = ob::StateSpacePtr(new ob::DubinsStateSpace(1., true));
		else
			robot_space = ob::StateSpacePtr(new ob::ReedsSheppStateSpace);

		runWithCarmen(argc, argv);
	}
	catch(std::exception& e) {
		std::cerr << "error: " << e.what() << "\n";
		return 1;
	}
	catch(...) {
		std::cerr << "Exception of unknown type!\n";
	}

	return 0;
}

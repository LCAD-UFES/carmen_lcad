
#include "carmen_comm.h"

#include <unistd.h>
#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/robot_ackerman_interface.h>
#include "g2o/types/slam2d/se2.h"


const double MAX_V = 20.0;
const double MAX_PHI = M_PI;
const double MAX_DELTA = 10.0; // max dx or dy between the goal and the pose


carmen_localize_ackerman_globalpos_message global_localize_ackerman_message;
carmen_behavior_selector_goal_list_message global_behavior_selector_goal_list_message;


void
publish_starting_pose(double x, double y, double th)
{
	carmen_localize_ackerman_globalpos_message localize_ackerman_message;
	memset(&localize_ackerman_message, 0, sizeof(localize_ackerman_message));

	carmen_point_t pose;
	carmen_point_t std;

	pose.x = x;
	pose.y = y;
	pose.theta = th;

	std.x = 0.001;
	std.y = 0.001;
	std.theta = carmen_degrees_to_radians(0.01);

	carmen_localize_ackerman_initialize_gaussian_command(pose, std);
}


void
publish_command(double v, double phi)
{
	static const int NUM_MOTION_COMMANDS = 20;
	static carmen_ackerman_motion_command_t *motion_commands = NULL;

	if (motion_commands == NULL)
		motion_commands = (carmen_ackerman_motion_command_t *) calloc (NUM_MOTION_COMMANDS, sizeof(carmen_ackerman_motion_command_t));

	for (int i = 0; i < NUM_MOTION_COMMANDS; i++)
	{
			motion_commands[i].time = 0.1 * i;
			motion_commands[i].v = v;
			motion_commands[i].phi = phi;
	}

	carmen_robot_ackerman_publish_motion_command(motion_commands,
		NUM_MOTION_COMMANDS, global_localize_ackerman_message.timestamp);
}


double
normalize(double x, double max_x)
{
	return x / max_x;
}


std::vector<double>
read_state()
{
	carmen_ipc_sleep(1e-4);
	std::vector<double> state;

	carmen_localize_ackerman_globalpos_message globalpos = global_localize_ackerman_message;
	carmen_behavior_selector_goal_list_message goals = global_behavior_selector_goal_list_message;

	g2o::SE2 t_pos(globalpos.globalpos.x, globalpos.globalpos.y, globalpos.globalpos.theta);
	g2o::SE2 t_goal(goals.goal_list[0].x, goals.goal_list[0].y, goals.goal_list[0].theta);
	g2o::SE2 dgoal = t_pos.inverse() * t_goal;

	state.push_back(normalize(dgoal[0], MAX_DELTA)); // x
	state.push_back(normalize(dgoal[1], MAX_DELTA)); // y
	state.push_back(normalize(dgoal[2], M_PI)); // theta

	state.push_back(normalize(globalpos.v, MAX_V));
	state.push_back(normalize(goals.goal_list[1].v, MAX_V));

	state.push_back(normalize(globalpos.phi, MAX_PHI));
	state.push_back(normalize(goals.goal_list[1].phi, MAX_PHI));

	return state;
}


void
env_destroy()
{
	publish_command(0, 0);
	carmen_ipc_disconnect();
	exit(0);
}


void
signal_handler(int signo)
{
	if (signo == SIGINT)
		env_destroy();
}


void
env_init()
{
	char *argv[] = {"rl_motion_planner"};
	carmen_ipc_initialize(1, argv);

    carmen_localize_ackerman_subscribe_globalpos_message(&global_localize_ackerman_message,
    	NULL, CARMEN_SUBSCRIBE_LATEST);

    carmen_behavior_selector_subscribe_goal_list_message(&global_behavior_selector_goal_list_message,
    	NULL, CARMEN_SUBSCRIBE_LATEST);

	signal(SIGINT, signal_handler);
}


std::vector<double>
env_reset(double x, double y, double th)
{
	memset(&global_localize_ackerman_message, 0, sizeof(carmen_localize_ackerman_globalpos_message));
	memset(&global_behavior_selector_goal_list_message, 0, sizeof(carmen_behavior_selector_goal_list_message));

	publish_command(0, 0);
	publish_starting_pose(x, y, th);

	while (global_localize_ackerman_message.timestamp == 0 ||
			global_behavior_selector_goal_list_message.timestamp == 0)
		carmen_ipc_sleep(1e-4);

	return read_state();
}


std::vector<double>
env_step(double v, double phi)
{
	usleep(1e4);
	publish_command(v, phi);
	return read_state();
}


bool
env_done()
{
	return false;
}


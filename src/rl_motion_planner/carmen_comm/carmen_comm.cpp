
#include "carmen_comm.h"

#include <unistd.h>
#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/robot_ackerman_interface.h>


carmen_localize_ackerman_globalpos_message global_localize_ackerman_message;


void
publish_starting_pose(float x, float y, float th)
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
publish_command(float v, float phi)
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


std::vector<float>
read_state()
{
	carmen_ipc_sleep(1e-4);

	std::vector<float> state;

	state.push_back(global_localize_ackerman_message.globalpos.x);
	state.push_back(global_localize_ackerman_message.globalpos.y);
	state.push_back(global_localize_ackerman_message.globalpos.theta);

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

	signal(SIGINT, signal_handler);
}


std::vector<float>
env_reset(float x, float y, float th)
{
	publish_command(0, 0);
	publish_starting_pose(x, y, th);

	return read_state();
}


std::vector<float>
env_step(float v, float phi)
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


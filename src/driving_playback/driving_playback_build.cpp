
#include <stdio.h>
#include <vector>
#include <float.h>
#include <iostream>
#include <signal.h>
#include <algorithm>
#include <sys/signal.h>

#include <carmen/carmen.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/carmen_gps_wrapper.h>

using namespace std;

FILE *carmen_driving_playback_build_file;

static char *carmen_driving_playback_filename;
static double carmen_driving_playback_min_distance_between_waypoints;

static void
carmen_driving_playback_build_shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect ();

		if (carmen_driving_playback_build_file != NULL)
			fclose(carmen_driving_playback_build_file);	

		fprintf (stderr, "\nDriving playback Disconnecting...\n");
		exit (0);
	}
}


static void
carmen_driving_playback_build_fused_odometry_handler (carmen_fused_odometry_message *message)
{
	double distance;
	carmen_point_t current_waypoint;
	static carmen_point_t last_waypoint;
	static int first_time = 1;

	current_waypoint.x = -message->pose.position.y;
	current_waypoint.y = message->pose.position.x;

	if (first_time)
	{
		distance = carmen_driving_playback_min_distance_between_waypoints;
		first_time = 0;
	}
	else
	{
		distance = carmen_distance(&current_waypoint, &last_waypoint);
	}

	if (distance < carmen_driving_playback_min_distance_between_waypoints)
		return;

	last_waypoint = current_waypoint;

	fprintf(carmen_driving_playback_build_file, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
		message->pose.position.x,
		message->pose.position.y,
		message->pose.position.z,
		message->pose.orientation.roll,
		message->pose.orientation.pitch,
		message->pose.orientation.yaw,
		message->velocity.x,
		message->velocity.y,
		message->velocity.z,
		message->phi,
		message->timestamp
	);

	fprintf(stdout, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
		message->pose.position.x,
		message->pose.position.y,
		message->pose.position.z,
		message->pose.orientation.roll,
		message->pose.orientation.pitch,
		message->pose.orientation.yaw,
		message->velocity.x,
		message->velocity.y,
		message->velocity.z,
		message->phi
	);
}


static void
carmen_driving_playback_build_get_parameters (int argc, char** argv)
{
	carmen_param_t param_list[] = {
			{(char *)"commandline", (char *)"file", CARMEN_PARAM_STRING, &carmen_driving_playback_filename, 0, NULL}
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	carmen_driving_playback_build_file = fopen(carmen_driving_playback_filename, "w");

	if (carmen_driving_playback_build_file == NULL)
		exit(printf("FILE '%s' NOT FOUND!\n", carmen_driving_playback_filename));

	// save all fused odometry information
	carmen_driving_playback_min_distance_between_waypoints = 0;
}


int
main (int argc, char **argv)
{
	carmen_driving_playback_build_file = NULL;

	setlocale (LC_ALL, "C");
	signal (SIGINT, carmen_driving_playback_build_shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	carmen_driving_playback_build_get_parameters(argc, argv);

	carmen_fused_odometry_subscribe_fused_odometry_message (NULL,
			(carmen_handler_t) carmen_driving_playback_build_fused_odometry_handler, CARMEN_SUBSCRIBE_ALL);

	carmen_ipc_dispatch();
	return (0);
}

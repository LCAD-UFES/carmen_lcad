#include <locale.h>
#include <signal.h>
#include <carmen/carmen.h>
#include "rddf_util.h"
#include <carmen/carmen_gps_wrapper.h>


static char *carmen_rddf_filename;
static double carmen_rddf_min_distance_between_waypoints;
static double carmen_rddf_max_velocity;


static void
carmen_rddf_build_shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_rddf_play_save_waypoints(carmen_rddf_filename);
		carmen_ipc_disconnect();
		fprintf(stderr, "\nRDDF Disconnecting...\n");
		exit(0);
	}
}


static void
carmen_rddf_build_get_parameters (int argc, char** argv)
{
	carmen_param_t param_list[] = {
			{(char *)"rddf", (char *)"min_distance_between_waypoints", CARMEN_PARAM_DOUBLE, &carmen_rddf_min_distance_between_waypoints, 1, NULL},
			{(char *)"robot", (char *)"max_velocity", CARMEN_PARAM_DOUBLE, &carmen_rddf_max_velocity, 1, NULL},
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	double latitude, longitude, altitude, distance;
	carmen_point_t current_waypoint;
	static carmen_point_t last_waypoint;
	static int first_time = 1;

	current_waypoint.x = -msg->globalpos.y;
	current_waypoint.y = msg->globalpos.x;

	if (first_time)
	{
		distance = carmen_rddf_min_distance_between_waypoints;
		first_time = 0;
	}
	else
	{
		distance = carmen_distance(&current_waypoint, &last_waypoint);
	}

	if (distance < carmen_rddf_min_distance_between_waypoints)
		return;

	last_waypoint = current_waypoint;

	carmen_Utm_Gdc3(last_waypoint.x, last_waypoint.y, 0, 24, 0, &latitude, &longitude, &altitude);

	carmen_rddf_play_add_waypoint(latitude, longitude);

	carmen_rddf_play_add_waypoint_speed(latitude, longitude,
			carmen_rddf_max_velocity,
			msg->v,
			msg->globalpos.theta,
			msg->timestamp);
}


void
carmen_rddf_timer_handler(char *carmen_rddf_filename)
{
	carmen_rddf_play_save_waypoints(carmen_rddf_filename);
}


int
main (int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Use %s <rddf-file>\n", argv[0]));

	carmen_rddf_filename = argv[1];

	setlocale(LC_ALL, "C");
	signal(SIGINT, carmen_rddf_build_shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	carmen_rddf_build_get_parameters(argc, argv);
	carmen_rddf_play_open_kml();

	carmen_localize_ackerman_subscribe_globalpos_message(
			NULL, (carmen_handler_t) localize_globalpos_handler,
			CARMEN_SUBSCRIBE_ALL);

	carmen_ipc_addPeriodicTimer(1.0, (TIMER_HANDLER_TYPE) carmen_rddf_timer_handler,
			carmen_rddf_filename);

	carmen_ipc_dispatch();
	return (0);
}

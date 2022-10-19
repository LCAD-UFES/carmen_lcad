#include <stdio.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <algorithm>
using namespace std;

#include <carmen/carmen.h>
#include <carmen/fused_odometry_interface.h>
#include "driving_playback_index.h"
#include <carmen/rddf_interface.h>
#include <carmen/motion_planner_interface.h>

FILE *carmen_driving_playback_file;

static char *carmen_driving_playback_filename = NULL;
static int carmen_driving_playback_num_poses_ahead_max = 0;
static double carmen_driving_playback_distance_between_waypoints = 0.0;

static carmen_robot_and_trailers_traj_point_t *carmen_driving_playback_poses_ahead;
static int carmen_driving_playback_num_poses_ahead;

int *annotations;

static carmen_fused_odometry_message carmen_driving_playback_fused_odometry;
static int carmen_driving_playback_pose_initialized = 0;


static void
carmen_driving_playback_play_shutdown_module (int signo)
{
	if (signo == SIGINT)
	{
		free(carmen_driving_playback_poses_ahead);
		carmen_ipc_disconnect();

		fprintf (stderr, "\nDriving playback Disconnecting...\n");
		exit(0);
	}
}


static int
carmen_driving_playback_play_find_nearest_poses_ahead(double x, double y, double yaw, double timestamp /* only for debugging */,
		carmen_robot_and_trailers_traj_point_t *poses_ahead, int num_poses_ahead_max, int *rddf_annotations, double distance_between_waypoints)
{
	return carmen_search_next_poses_index(x, y, yaw, timestamp, poses_ahead, num_poses_ahead_max, rddf_annotations,  distance_between_waypoints);
}


void
carmen_driving_playback_play_publish_commands()
{
/*
	IPC_RETURN_TYPE err;
	carmen_rddf_road_profile_message rddf_road_profile_message;

	rddf_road_profile_message.poses = carmen_driving_playback_poses_ahead;
	rddf_road_profile_message.number_of_poses = carmen_driving_playback_num_poses_ahead;
	rddf_road_profile_message.annotations = annotations;
	rddf_road_profile_message.signals_annotations = annotations;
	rddf_road_profile_message.timestamp = carmen_get_time();
	rddf_road_profile_message.host = carmen_get_host();

	err = IPC_publishData(CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME, &rddf_road_profile_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_ROAD_PROFILE_MESSAGE_FMT);
*/
	IPC_RETURN_TYPE err;

	carmen_motion_planner_path_message message;

	message.path = carmen_driving_playback_poses_ahead;
	message.path_size = carmen_driving_playback_num_poses_ahead;

	message.algorithm = 0;

	message.timestamp = carmen_get_time();
	message.host = carmen_get_host();

	err = IPC_publishData(CARMEN_MOTION_PLANNER_PATH_NAME, &message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MOTION_PLANNER_PATH_FMT);

	//printf("publish %d poses\n", carmen_driving_playback_num_poses_ahead);
}


static void
carmen_driving_playback_play_fused_odometry_handler()
{
	if (!carmen_driving_playback_pose_initialized)
		carmen_driving_playback_pose_initialized = 1;

	carmen_driving_playback_num_poses_ahead = carmen_driving_playback_play_find_nearest_poses_ahead(
			carmen_driving_playback_fused_odometry.pose.position.x,
			carmen_driving_playback_fused_odometry.pose.position.y,
			carmen_driving_playback_fused_odometry.pose.orientation.yaw,
			carmen_driving_playback_fused_odometry.timestamp,
			carmen_driving_playback_poses_ahead,
			carmen_driving_playback_num_poses_ahead_max,
			annotations,
			carmen_driving_playback_distance_between_waypoints
	);

	carmen_driving_playback_play_publish_commands();
}


void
carmen_driving_playback_play_subscribe_messages()
{
	carmen_fused_odometry_subscribe_fused_odometry_message(&carmen_driving_playback_fused_odometry, (carmen_handler_t) carmen_driving_playback_play_fused_odometry_handler, CARMEN_SUBSCRIBE_LATEST);
}

static void
carmen_driving_playback_play_load_index(char *carmen_driving_playback_filename)
{
	carmen_fused_odometry_message message;

	if (!carmen_driving_playback_index_exists(carmen_driving_playback_filename))
	{
		carmen_driving_playback_file = fopen(carmen_driving_playback_filename, "r");

		while(!feof(carmen_driving_playback_file))
		{
			fscanf(carmen_driving_playback_file, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
				&(message.pose.position.x),
				&(message.pose.position.y),
				&(message.pose.position.z),
				&(message.pose.orientation.roll),
				&(message.pose.orientation.pitch),
				&(message.pose.orientation.yaw),
				&(message.velocity.x),
				&(message.velocity.y),
				&(message.velocity.z),
				&(message.phi),
				&(message.timestamp)
			);

			carmen_driving_playback_index_add(&message, 0, 0, 0);
		}

		carmen_driving_playback_index_save(carmen_driving_playback_filename);

		fclose(carmen_driving_playback_file);
	}

	carmen_driving_playback_load_index(carmen_driving_playback_filename);
}


static void
carmen_driving_playback_play_get_parameters(int argc, char** argv)
{
	carmen_param_t param_list[] =
	{
			{(char *) "driving_playback", (char *) "num_poses_ahead", CARMEN_PARAM_INT, &carmen_driving_playback_num_poses_ahead_max, 0, NULL},
			{(char *) "driving_playback", (char *) "distance_between_waypoints", CARMEN_PARAM_DOUBLE, &carmen_driving_playback_distance_between_waypoints, 0, NULL},
			{(char *)"commandline", (char *)"file", CARMEN_PARAM_STRING, &carmen_driving_playback_filename, 0, NULL}
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}


int
main(int argc, char **argv)
{
	setlocale (LC_ALL, "C");

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	carmen_motion_planner_define_path_message();
	carmen_driving_playback_play_get_parameters(argc, argv);

	carmen_driving_playback_poses_ahead = (carmen_robot_and_trailers_traj_point_t *) calloc (carmen_driving_playback_num_poses_ahead_max, sizeof(carmen_robot_and_trailers_traj_point_t));
	annotations = (int *) calloc (carmen_driving_playback_num_poses_ahead_max, sizeof(int));

	carmen_test_alloc(carmen_driving_playback_poses_ahead);
	carmen_test_alloc(annotations);

	signal (SIGINT, carmen_driving_playback_play_shutdown_module);

	carmen_driving_playback_play_load_index(carmen_driving_playback_filename);
	carmen_driving_playback_play_subscribe_messages();
	carmen_ipc_dispatch ();

	return (0);
}

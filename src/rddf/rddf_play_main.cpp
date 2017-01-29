#include <stdio.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <algorithm>
using namespace std;

#include <carmen/carmen.h>
#include <carmen/readlog.h>
#include <carmen/carmen_stdio.h>
#include <carmen/velodyne_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/carmen_gps_wrapper.h>

#include "rddf_interface.h"
#include "rddf_messages.h"
#include "rddf_index.h"

#include <kml/base/file.h>
#include <kml/engine.h>
#include <kml/dom.h>
#include <kml/engine/kml_file.h>
#include "g2o/types/slam2d/se2.h"

using namespace g2o;

typedef std::vector<kmldom::PlacemarkPtr> placemark_vector_t;

#include "rddf_util.h"

static char *carmen_rddf_filename = NULL;

static int carmen_rddf_perform_loop = 0;
static int carmen_rddf_num_poses_ahead_max = 100;

static int carmen_rddf_end_point_is_set = 0;
static carmen_point_t carmen_rddf_end_point;

static int carmen_rddf_nearest_waypoint_is_set = 0;
static carmen_point_t carmen_rddf_nearest_waypoint_to_end_point;

static carmen_ackerman_traj_point_t *carmen_rddf_poses_ahead = NULL;
static carmen_ackerman_traj_point_t *carmen_rddf_poses_back = NULL;
static int carmen_rddf_num_poses_ahead = 0;
static int carmen_rddf_num_poses_back = 0;
static int *annotations;

static carmen_point_t robot_pose;
static int carmen_rddf_pose_initialized = 0;
static int already_reached_nearest_waypoint_to_end_point = 0;

char *carmen_annotation_filename = NULL;
vector<carmen_rddf_add_annotation_message> annotation_queue;
vector<int> annotations_to_publish;

int use_truepos = 0;


static void
carmen_rddf_play_shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		exit(0);
	}
}


int
carmen_rddf_play_nearest_waypoint_reached(carmen_ackerman_traj_point_t pose)
{
	if (sqrt(pow(pose.x - carmen_rddf_nearest_waypoint_to_end_point.x, 2) + pow(pose.y - carmen_rddf_nearest_waypoint_to_end_point.y, 2)) < 2.0)
		return 1;
	else
		return 0;
}


int
carmen_rddf_play_find_position_of_nearest_waypoint(carmen_ackerman_traj_point_t *poses_ahead, int num_poses)
{
	int i, position = -1;

	for (i = 0; i < num_poses; i++)
	{
		if (carmen_rddf_play_nearest_waypoint_reached(poses_ahead[i]))
		{
			position = i;
			break;
		}
	}

	return position;
}


int
carmen_rddf_play_adjust_poses_ahead_and_add_end_point_to_list(carmen_ackerman_traj_point_t *poses_ahead, int num_poses, int nearest_end_waypoint_position, int *rddf_annotations)
{
	int position_nearest_waypoint = nearest_end_waypoint_position;
	int position_end_point = nearest_end_waypoint_position + 1;

	//
	// se o waypoint mais proximo ao end point for o ultimo da lista,
	// a sua posicao eh decrementada para abrir espaco para o end point
	//

	if (nearest_end_waypoint_position == (num_poses - 1))
	{
		position_nearest_waypoint--;
		position_end_point--;
	}

	poses_ahead[position_nearest_waypoint].x = carmen_rddf_nearest_waypoint_to_end_point.x;
	poses_ahead[position_nearest_waypoint].y = carmen_rddf_nearest_waypoint_to_end_point.y;
	rddf_annotations[position_nearest_waypoint] = RDDF_ANNOTATION_NONE;

	poses_ahead[position_end_point].x = carmen_rddf_end_point.x;
	poses_ahead[position_end_point].y = carmen_rddf_end_point.y;
	poses_ahead[position_end_point].theta = carmen_rddf_end_point.theta;
	poses_ahead[position_end_point].v = 0.0;
	rddf_annotations[position_end_point] = RDDF_ANNOTATION_END_POINT_AREA;

	return (position_end_point + 1);
}


int
carmen_rddf_play_check_if_end_point_is_reachable(carmen_ackerman_traj_point_t *poses_ahead, int num_poses, int *rddf_annotations)
{
	if (carmen_rddf_nearest_waypoint_is_set)
	{
		// se o robo ja passou pelo waypoint mais proximo do end point, so o end point eh publicado
		if (already_reached_nearest_waypoint_to_end_point)
		{
			poses_ahead[0].x = carmen_rddf_end_point.x;
			poses_ahead[0].y = carmen_rddf_end_point.y;
			poses_ahead[0].theta = carmen_rddf_end_point.theta;
			poses_ahead[0].v = 0.0;
			rddf_annotations[0] = RDDF_ANNOTATION_END_POINT_AREA;

			return 1;
		}

		// verifica se algum dos waypoints esta a uma distancia minima do waypoint mais proximo do end point
		int nearest_end_waypoint_position = carmen_rddf_play_find_position_of_nearest_waypoint (poses_ahead, num_poses);

		if (nearest_end_waypoint_position != -1)
		{
			// se um dos dois primeiros waypoints esta a uma distancia minima do waypoint mais proximo do end point, passamos a publicar somente o end point
			if (nearest_end_waypoint_position < 2)
				already_reached_nearest_waypoint_to_end_point = 1;

			return carmen_rddf_play_adjust_poses_ahead_and_add_end_point_to_list(poses_ahead, num_poses, nearest_end_waypoint_position, rddf_annotations);
		}
		else
			return num_poses;
	}
	else
		return num_poses;
}


static void
clear_annotations(int *rddf_annotations, int num_annotations)
{
	int i;

	for(i = 0; i < num_annotations; i++)
		rddf_annotations[i] = RDDF_ANNOTATION_NONE;
}


void
clear_annotations()
{
	for (int i = 0; i < carmen_rddf_num_poses_ahead; i++)
		annotations[i] = RDDF_ANNOTATION_NONE;
}


static int
carmen_rddf_play_find_nearest_poses_ahead(double x, double y, double yaw, double timestamp /* only for debugging */, carmen_ackerman_traj_point_t *poses_ahead, carmen_ackerman_traj_point_t *poses_back, int *num_poses_back, int num_poses_ahead_max, int *rddf_annotations)
{
	clear_annotations(rddf_annotations, num_poses_ahead_max);

	int num_poses_ahead = carmen_search_next_poses_index(x, y, yaw, timestamp, poses_ahead, poses_back, num_poses_back, num_poses_ahead_max, rddf_annotations, carmen_rddf_perform_loop);
	return carmen_rddf_play_check_if_end_point_is_reachable(poses_ahead, num_poses_ahead, rddf_annotations);
}


void
carmen_check_for_annotations(double x, double y, double theta __attribute__((unused)))
{
	double dx, dy, dist;

	for (size_t i = 0; i < annotation_queue.size(); i++)
	{
		dx = annotation_queue[i].annotation_point.x - x;
		dy = annotation_queue[i].annotation_point.y - y;
		dist = sqrt(pow(dx, 2) + pow(dy, 2));

		// *******************************************************************
		// TODO: criar uma forma de buscar somente as anotacoes na mao correta
		// *******************************************************************

		if (dist < 300.0)// TODO: Alberto: Isso depende da velocidade do carro... Quando a velocidade eh maior que 60Km/h este valor esta ruim...
			annotations_to_publish.push_back(i);
	}
}


void
find_nearest_pose_and_dist(int annotation_id, int *nearest_pose_out, double *nearest_pose_dist_out)
{
	int nearest_pose;
	double min_distance_to_annotation;
	double distance_to_annotation;
	carmen_rddf_add_annotation_message annotation;

	nearest_pose = -1;
	min_distance_to_annotation = DBL_MAX;

	annotation = annotation_queue[annotation_id];

	for (int i = 0; i < carmen_rddf_num_poses_ahead; i++)
	{
		distance_to_annotation = sqrt(pow(carmen_rddf_poses_ahead[i].x - annotation.annotation_point.x, 2) + pow(carmen_rddf_poses_ahead[i].y - annotation.annotation_point.y, 2));

		if (distance_to_annotation < min_distance_to_annotation)
		{
			min_distance_to_annotation = distance_to_annotation;
			nearest_pose = i;
		}
	}

	(*nearest_pose_out) = nearest_pose;
	(*nearest_pose_dist_out) = min_distance_to_annotation;
}


int
annotation_is_forward_from_robot(carmen_point_t pose, int annotation_id)
{
	carmen_vector_3D_t annotation_point;
	carmen_rddf_add_annotation_message annotation;

	annotation = annotation_queue[annotation_id];
	annotation_point = annotation.annotation_point;

	SE2 robot_pose_mat(pose.x, pose.y, pose.theta);
	SE2 annotation_point_mat(annotation_point.x, annotation_point.y, 0.0);
	SE2 annotation_in_car_reference = robot_pose_mat.inverse() * annotation_point_mat;

	if (annotation_in_car_reference[0] > 0.0)
		return 1;
	else
		return 0;
}


void
set_annotations()
{
	int nearest_pose;
	double nearest_pose_dist;

	for (uint i = 0; i < annotations_to_publish.size(); i++)
	{
		find_nearest_pose_and_dist(annotations_to_publish[i], &nearest_pose, &nearest_pose_dist);

		if ((nearest_pose >= 0) && nearest_pose_dist < 10.0 && (annotation_is_forward_from_robot(robot_pose, annotations_to_publish[i])))
			annotations[nearest_pose] = annotation_queue[annotations_to_publish[i]].annotation_type;
	}
}


static void
carmen_rddf_play_publish_rddf()
{
	// so publica rddfs quando a pose do robo ja estiver setada
	if ((carmen_rddf_num_poses_ahead > 0) && (carmen_rddf_num_poses_back > 0))
	{
		clear_annotations();
		set_annotations();

		carmen_rddf_publish_road_profile_message(
			carmen_rddf_poses_ahead,
			carmen_rddf_poses_back,
			carmen_rddf_num_poses_ahead,
			carmen_rddf_num_poses_back,
			annotations);

		/* publica as anotacoes */
		IPC_RETURN_TYPE err;

		for (size_t i = 0; i < annotations_to_publish.size(); i++)
		{
			annotation_queue[annotations_to_publish[i]].timestamp = carmen_get_time();
			err = IPC_publishData(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, &annotation_queue[annotations_to_publish[i]]);
			carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_ANNOTATION_MESSAGE_FMT);
		}
	}
}


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	robot_pose = msg->globalpos;
	carmen_rddf_pose_initialized = 1;

	carmen_rddf_num_poses_ahead = carmen_rddf_play_find_nearest_poses_ahead(
			robot_pose.x,
			robot_pose.y,
			robot_pose.theta,
			msg->timestamp,
			carmen_rddf_poses_ahead,
			carmen_rddf_poses_back,
			&carmen_rddf_num_poses_back,
			carmen_rddf_num_poses_ahead_max,
			annotations
	);

	annotations_to_publish.clear();
	carmen_check_for_annotations(robot_pose.x, robot_pose.y, robot_pose.theta);

	carmen_rddf_play_publish_rddf();
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	robot_pose = msg->truepose;
	carmen_rddf_pose_initialized = 1;

	carmen_rddf_num_poses_ahead = carmen_rddf_play_find_nearest_poses_ahead(
			robot_pose.x,
			robot_pose.y,
			robot_pose.theta,
			msg->timestamp,
			carmen_rddf_poses_ahead,
			carmen_rddf_poses_back,
			&carmen_rddf_num_poses_back,
			carmen_rddf_num_poses_ahead_max,
			annotations
	);

	annotations_to_publish.clear();
	carmen_check_for_annotations(robot_pose.x, robot_pose.y, robot_pose.theta);

	carmen_rddf_play_publish_rddf();
}


static void
carmen_rddf_play_nearest_waypoint_message_handler(carmen_rddf_nearest_waypoint_message *rddf_nearest_waypoint_message)
{
	carmen_rddf_nearest_waypoint_to_end_point = rddf_nearest_waypoint_message->point;
	carmen_rddf_nearest_waypoint_is_set = 1;

	carmen_rddf_publish_nearest_waypoint_confirmation_message(rddf_nearest_waypoint_message->point);
	already_reached_nearest_waypoint_to_end_point = 0;
}


void
carmen_rddf_play_find_and_publish_poses_around_end_point(double x, double y, double yaw, int num_poses_desired, double timestamp)
{
	int num_poses_acquired = 0;
	carmen_ackerman_traj_point_t *poses_around_end_point;

	poses_around_end_point = (carmen_ackerman_traj_point_t *) calloc (num_poses_desired, sizeof(carmen_ackerman_traj_point_t));
	carmen_test_alloc(poses_around_end_point);

	num_poses_acquired = carmen_find_poses_around(x, y, yaw, timestamp, poses_around_end_point, num_poses_desired);
	carmen_rddf_publish_road_profile_around_end_point_message(poses_around_end_point, num_poses_acquired);

	free(poses_around_end_point);
}


void
carmen_rddf_play_end_point_message_handler(carmen_rddf_end_point_message *rddf_end_point_message)
{
	if (rddf_end_point_message->number_of_poses > 1)
	{
		carmen_rddf_play_find_and_publish_poses_around_end_point(
				rddf_end_point_message->point.x,
				rddf_end_point_message->point.y,
				rddf_end_point_message->point.theta,
				rddf_end_point_message->number_of_poses,
				rddf_end_point_message->timestamp
		);

		carmen_rddf_end_point = rddf_end_point_message->point;
		carmen_rddf_end_point_is_set = 1;
	}
	else
	{
		carmen_rddf_play_find_and_publish_poses_around_end_point(
				rddf_end_point_message->point.x,
				rddf_end_point_message->point.y,
				rddf_end_point_message->point.theta,
				rddf_end_point_message->number_of_poses,
				rddf_end_point_message->timestamp
		);

		carmen_rddf_nearest_waypoint_to_end_point = rddf_end_point_message->point;
		carmen_rddf_nearest_waypoint_is_set = 1;

		already_reached_nearest_waypoint_to_end_point = 0;
	}
}


void
carmen_rddf_play_subscribe_messages()
{
	if (!use_truepos)
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_nearest_waypoint_message(NULL,
			(carmen_handler_t) carmen_rddf_play_nearest_waypoint_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_end_point_message(NULL,
			(carmen_handler_t) carmen_rddf_play_end_point_message_handler,
			CARMEN_SUBSCRIBE_LATEST);
}


static void
carmen_rddf_play_load_index(char *rddf_filename)
{
	int annotation = 0;
	carmen_fused_odometry_message message;
	placemark_vector_t placemark_vector;

	if (!carmen_rddf_index_exists(rddf_filename))
	{
		carmen_rddf_play_open_kml(rddf_filename, &placemark_vector);

		for (unsigned int i = 0; i < placemark_vector.size(); i++)
		{
			if (carmen_rddf_play_copy_kml(placemark_vector[i], &message, &annotation))
				carmen_rddf_index_add(&message, 0, 0, annotation);
		}

		carmen_rddf_index_save(rddf_filename);
	}

	carmen_rddf_load_index(rddf_filename);
}


void
carmen_rddf_play_load_annotation_file()
{
	if (carmen_annotation_filename == NULL)
		return;

	FILE *f = fopen(carmen_annotation_filename, "r");

	while(!feof(f))
	{
		carmen_rddf_annotation_message message;
		message.annotation_description = (char *) calloc (64, sizeof(char));

		fscanf(f, "%s\t%d\t%d\t%lf\t%lf\t%lf\t%lf\n",
			message.annotation_description,
			&message.annotation_type,
			&message.annotation_code,
			&message.annotation_orientation,
			&message.annotation_point.x,
			&message.annotation_point.y,
			&message.annotation_point.z
		);

		message.host = carmen_get_host();
		annotation_queue.push_back(message);
	}

	fclose(f);
}


static void
carmen_rddf_play_get_parameters(int argc, char** argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "rddf", (char *) "num_poses_ahead", CARMEN_PARAM_INT, &carmen_rddf_num_poses_ahead_max, 0, NULL},
		{(char *) "rddf", (char *) "loop", CARMEN_PARAM_ONOFF, &carmen_rddf_perform_loop, 0, NULL},
		{(char *) "behavior_selector", (char *) "use_truepos", CARMEN_PARAM_ONOFF, &use_truepos, 0, NULL}
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}


void
carmen_rddf_play_initialize(void)
{
	carmen_rddf_poses_ahead = (carmen_ackerman_traj_point_t *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(carmen_ackerman_traj_point_t));
	carmen_rddf_poses_back = (carmen_ackerman_traj_point_t *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(carmen_ackerman_traj_point_t));
	annotations = (int *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(int));

	carmen_test_alloc(carmen_rddf_poses_ahead);
	carmen_test_alloc(annotations);

	// publish rddf at a rate of 10Hz
	// carmen_ipc_addPeriodicTimer(1.0 / 10.0, (TIMER_HANDLER_TYPE) carmen_rddf_play_publish_rddf, NULL);
}


int
main(int argc, char **argv)
{
	setlocale(LC_ALL, "C");

	if (argc < 2)
		exit(printf("Error: Use %s <rddf> <annotations>\n", argv[0]));

	carmen_rddf_filename = argv[1];

	/* annotations file are not mandatory */
	if (argc == 3)
		carmen_annotation_filename = argv[2];
	else
		carmen_annotation_filename = NULL;

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	carmen_rddf_play_get_parameters(argc, argv);
	carmen_rddf_play_initialize();
	carmen_rddf_define_messages();
	carmen_rddf_play_subscribe_messages();
	carmen_rddf_play_load_index(carmen_rddf_filename);
	carmen_rddf_play_load_annotation_file();

	signal (SIGINT, carmen_rddf_play_shutdown_module);
	carmen_ipc_dispatch();

	return (0);
}

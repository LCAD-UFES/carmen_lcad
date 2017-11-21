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
#include <carmen/traffic_light_interface.h>
#include <carmen/traffic_light_messages.h>
#include <carmen/collision_detection.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>

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
static double distance_between_front_and_rear_axles;
static double distance_between_front_car_and_front_wheels;

static int carmen_rddf_end_point_is_set = 0;
static carmen_point_t carmen_rddf_end_point;

static int carmen_rddf_nearest_waypoint_is_set = 0;
static carmen_point_t carmen_rddf_nearest_waypoint_to_end_point;

static carmen_ackerman_traj_point_t *carmen_rddf_poses_ahead = NULL;
static carmen_ackerman_traj_point_t *carmen_rddf_poses_back = NULL;
static int carmen_rddf_num_poses_ahead = 0;
static int carmen_rddf_num_poses_back = 0;
static int *annotations_codes;
static int *annotations;

static int carmen_rddf_pose_initialized = 0;
static int already_reached_nearest_waypoint_to_end_point = 0;

char *carmen_annotation_filename = NULL;
vector<carmen_annotation_t> annotation_read_from_file;
typedef struct
{
	carmen_annotation_t annotation;
	size_t index;
} annotation_and_index;
vector<annotation_and_index> annotations_to_publish;
carmen_rddf_annotation_message annotation_queue_message;

static int use_truepos = 0;

static int traffic_lights_camera = 3;
carmen_traffic_light_message *traffic_lights = NULL;

deque<carmen_rddf_dynamic_annotation_message> dynamic_annotation_messages;

carmen_moving_objects_point_clouds_message *moving_objects = NULL;


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
	rddf_annotations[position_nearest_waypoint] = RDDF_ANNOTATION_TYPE_NONE;

	poses_ahead[position_end_point].x = carmen_rddf_end_point.x;
	poses_ahead[position_end_point].y = carmen_rddf_end_point.y;
	poses_ahead[position_end_point].theta = carmen_rddf_end_point.theta;
	poses_ahead[position_end_point].v = 0.0;
	rddf_annotations[position_end_point] = RDDF_ANNOTATION_TYPE_END_POINT_AREA;

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
			rddf_annotations[0] = RDDF_ANNOTATION_TYPE_END_POINT_AREA;

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
	{
		rddf_annotations[i] = RDDF_ANNOTATION_TYPE_NONE;
		annotations_codes[i] = RDDF_ANNOTATION_CODE_NONE;
	}
}


void
clear_annotations()
{
	for (int i = 0; i < carmen_rddf_num_poses_ahead; i++)
	{
		annotations[i] = RDDF_ANNOTATION_TYPE_NONE;
		annotations_codes[i] = RDDF_ANNOTATION_CODE_NONE;
	}
}


static int
carmen_rddf_play_find_nearest_poses_ahead(double x, double y, double yaw, double timestamp /* only for debugging */, carmen_ackerman_traj_point_t *poses_ahead, carmen_ackerman_traj_point_t *poses_back, int *num_poses_back, int num_poses_ahead_max, int *rddf_annotations)
{
	clear_annotations(rddf_annotations, num_poses_ahead_max);

	int num_poses_ahead = carmen_search_next_poses_index(x, y, yaw, timestamp, poses_ahead, poses_back, num_poses_back, num_poses_ahead_max, rddf_annotations, carmen_rddf_perform_loop);
	return carmen_rddf_play_check_if_end_point_is_reachable(poses_ahead, num_poses_ahead, rddf_annotations);
}


bool
pedestrian_track_busy(carmen_moving_objects_point_clouds_message *moving_objects, carmen_annotation_t pedestrain_track_annotation)
{
	carmen_vector_2D_t world_point;
	double displacement = distance_between_front_and_rear_axles + distance_between_front_car_and_front_wheels;
	double theta = pedestrain_track_annotation.annotation_orientation;
	world_point.x = pedestrain_track_annotation.annotation_point.x + displacement * cos(theta);
	world_point.y = pedestrain_track_annotation.annotation_point.y + displacement * sin(theta);

	for (int i = 0; i < moving_objects->num_point_clouds; i++)
	{
		if ((strcmp(moving_objects->point_clouds[i].model_features.model_name, "pedestrian") == 0) &&
			(DIST2D(moving_objects->point_clouds[i].object_pose, world_point) < pedestrain_track_annotation.annotation_point.z))
			return (true);
	}
	return (false);
}


bool
add_annotation(double x, double y, double theta, size_t annotation_index)
{
	double dx = annotation_read_from_file[annotation_index].annotation_point.x - x;
	double dy = annotation_read_from_file[annotation_index].annotation_point.y - y;
	double dist = sqrt(pow(dx, 2) + pow(dy, 2));
	double angle_to_annotation = carmen_radians_to_degrees(fabs(carmen_normalize_theta(theta - annotation_read_from_file[annotation_index].annotation_orientation)));

	if (annotation_read_from_file[annotation_index].annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT)
	{
		bool orientation_ok = angle_to_annotation < 70.0 ? true : false;

		if ((dist < MAX_TRAFFIC_LIGHT_DISTANCE) && orientation_ok)
		{
			if ((traffic_lights != NULL) &&
				(traffic_lights->num_traffic_lights > 0)) // @@@ Alberto: deveria verificar a maioria...
			{
				int num_red = 0;
				for (int i = 0; i < traffic_lights->num_traffic_lights; i++)
					if (traffic_lights->traffic_lights[i].color == RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED)
						num_red++;

				annotation_and_index annotation_i = {annotation_read_from_file[annotation_index], annotation_index};
				if (num_red > 0)
					annotation_i.annotation.annotation_code = RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED;
				else
					annotation_i.annotation.annotation_code = traffic_lights->traffic_lights[0].color;
				annotations_to_publish.push_back(annotation_i);
			}
			else
			{
				annotation_and_index annotation_i = {annotation_read_from_file[annotation_index], annotation_index};
				annotations_to_publish.push_back(annotation_i);
			}
			return (true);
		}
	}
	else if (annotation_read_from_file[annotation_index].annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK)
	{
		bool orientation_ok = angle_to_annotation < 70.0 ? true : false;

		if ((dist < 100.0) && orientation_ok)
		{
			if (moving_objects != NULL)
			{
				annotation_and_index annotation_i = {annotation_read_from_file[annotation_index], annotation_index};
				if (pedestrian_track_busy(moving_objects, annotation_read_from_file[annotation_index]))
					annotation_i.annotation.annotation_code = RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_BUSY;
				else
					annotation_i.annotation.annotation_code = RDDF_ANNOTATION_CODE_NONE;
				annotations_to_publish.push_back(annotation_i);
			}
			else
			{
				annotation_and_index annotation_i = {annotation_read_from_file[annotation_index], annotation_index};
				annotations_to_publish.push_back(annotation_i);
			}
			return (true);
		}
	}
	else if (annotation_read_from_file[annotation_index].annotation_type == RDDF_ANNOTATION_TYPE_STOP)
	{
		bool orientation_ok = angle_to_annotation < 15.0 ? true : false;

		if ((dist < 20.0) && orientation_ok)
		{
			annotation_and_index annotation_i = {annotation_read_from_file[annotation_index], annotation_index};
			annotations_to_publish.push_back(annotation_i);
			return (true);
		}
	}
	else if (dist < 20.0)
	{
		bool orientation_ok = angle_to_annotation < 70.0 ? true : false;

		if (orientation_ok)
		{
			annotation_and_index annotation_i = {annotation_read_from_file[annotation_index], annotation_index};
			annotations_to_publish.push_back(annotation_i);
		}
		return (true);
	}

	return (false);
}


void
carmen_check_for_annotations(carmen_point_t robot_pose,
		carmen_ackerman_traj_point_t *carmen_rddf_poses_ahead, carmen_ackerman_traj_point_t *carmen_rddf_poses_back,
		int carmen_rddf_num_poses_ahead, int carmen_rddf_num_poses_back, double timestamp)
{
	for (size_t annotation_index = 0; annotation_index < annotation_read_from_file.size(); annotation_index++)
	{
		if (add_annotation(robot_pose.x, robot_pose.y, robot_pose.theta, annotation_index))
			continue;

		bool added = false;
		for (int j = 0; j < carmen_rddf_num_poses_ahead; j++)
		{
			if (add_annotation(carmen_rddf_poses_ahead[j].x, carmen_rddf_poses_ahead[j].y, carmen_rddf_poses_ahead[j].theta, annotation_index))
			{
				added = true;
				break;
			}
		}

		if (!added)
		{
			for (int j = 0; j < carmen_rddf_num_poses_back; j++)
				if (add_annotation(carmen_rddf_poses_back[j].x, carmen_rddf_poses_back[j].y, carmen_rddf_poses_back[j].theta, annotation_index))
					break;
		}
	}

	for (size_t j = 0; j < dynamic_annotation_messages.size(); j++)
	{
		if ((timestamp - dynamic_annotation_messages[j].timestamp) < 2.0)
		{
			carmen_annotation_t annotation;
			annotation.annotation_type = dynamic_annotation_messages[j].annotation_type;
			annotation.annotation_code = dynamic_annotation_messages[j].annotation_code;
			annotation.annotation_point = dynamic_annotation_messages[j].annotation_point;
			annotation.annotation_description = dynamic_annotation_messages[j].annotation_description;
			annotation.annotation_orientation = dynamic_annotation_messages[j].annotation_orientation;
			annotation_and_index annotation_i = {annotation, 0};
			annotations_to_publish.push_back(annotation_i);
		}
		else
			dynamic_annotation_messages.erase(dynamic_annotation_messages.begin() + j);
	}
}


void
find_nearest_waypoint_and_dist(carmen_annotation_t annotation, int *nearest_pose_out, double *nearest_pose_dist_out)
{
	int nearest_pose;
	double min_distance_to_annotation;
	double distance_to_annotation;

	nearest_pose = -1;
	min_distance_to_annotation = DBL_MAX;

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
annotation_is_forward_from_robot(carmen_point_t pose, carmen_annotation_t annotation)
{
	carmen_vector_3D_t annotation_point;

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
set_annotations(carmen_point_t robot_pose)
{
	int nearest_pose;
	double nearest_pose_dist;

	for (uint i = 0; i < annotations_to_publish.size(); i++)
	{
		find_nearest_waypoint_and_dist(annotations_to_publish[i].annotation, &nearest_pose, &nearest_pose_dist);

		if ((nearest_pose >= 0) && nearest_pose_dist < 10.0 && (annotation_is_forward_from_robot(robot_pose, annotations_to_publish[i].annotation)))
		{
			annotations[nearest_pose] = annotations_to_publish[i].annotation.annotation_type;
			annotations_codes[nearest_pose] = annotations_to_publish[i].annotation.annotation_code;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_rddf_play_publish_annotation_queue()
{
	IPC_RETURN_TYPE err;

	if (annotations_to_publish.size() == 0)
	{
		annotation_and_index annotation_i;
		memset(&annotation_i, 0, sizeof(annotation_and_index));
		carmen_annotation_t &annotation = annotation_i.annotation;
		annotation.annotation_type = RDDF_ANNOTATION_TYPE_NONE;
		annotation.annotation_code = RDDF_ANNOTATION_CODE_NONE;
		annotations_to_publish.push_back(annotation_i);
	}

	if (annotation_queue_message.annotations == NULL)
	{
		annotation_queue_message.annotations = (carmen_annotation_t *) calloc(annotations_to_publish.size(), sizeof(carmen_annotation_t));

		if (!annotation_queue_message.annotations)
			exit(printf("Allocation error in carmen_rddf_play_publish_annotation_queue()::1\n"));
	}
	else if (annotation_queue_message.num_annotations != (int) annotations_to_publish.size())
	{
		annotation_queue_message.annotations = (carmen_annotation_t *) realloc(annotation_queue_message.annotations, annotations_to_publish.size() * sizeof(carmen_annotation_t));

		if (!annotation_queue_message.annotations)
			exit(printf("Allocation error in carmen_rddf_play_publish_annotation_queue()::2\n"));
	}

	annotation_queue_message.num_annotations = annotations_to_publish.size();

	for (size_t i = 0; i < annotations_to_publish.size(); i++)
		memcpy(&(annotation_queue_message.annotations[i]), &(annotations_to_publish[i].annotation), sizeof(carmen_annotation_t));

	annotation_queue_message.host = carmen_get_host();
	annotation_queue_message.timestamp = carmen_get_time();

	err = IPC_publishData(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, &annotation_queue_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_ANNOTATION_MESSAGE_FMT);
}


static void
carmen_rddf_play_publish_rddf_and_annotations(carmen_point_t robot_pose)
{
	// so publica rddfs quando a pose do robo ja estiver setada
	if ((carmen_rddf_num_poses_ahead > 0) && (carmen_rddf_num_poses_back > 0))
	{
		clear_annotations();
		set_annotations(robot_pose);

		carmen_rddf_publish_road_profile_message(
			carmen_rddf_poses_ahead,
			carmen_rddf_poses_back,
			carmen_rddf_num_poses_ahead,
			carmen_rddf_num_poses_back,
			annotations,
			annotations_codes);

		carmen_rddf_play_publish_annotation_queue();
	}
}


void
carmen_rddf_publish_road_profile_around_end_point(carmen_ackerman_traj_point_t *poses_around_end_point, int num_poses_acquired)
{
	carmen_rddf_publish_road_profile_around_end_point_message(poses_around_end_point, num_poses_acquired);
}
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_rddf_play_find_and_publish_poses_around_end_point(double x, double y, double yaw, int num_poses_desired, double timestamp)
{
	int num_poses_acquired = 0;
	carmen_ackerman_traj_point_t *poses_around_end_point;

	poses_around_end_point = (carmen_ackerman_traj_point_t *) calloc (num_poses_desired, sizeof(carmen_ackerman_traj_point_t));
	carmen_test_alloc(poses_around_end_point);

	num_poses_acquired = carmen_find_poses_around(x, y, yaw, timestamp, poses_around_end_point, num_poses_desired);
	carmen_rddf_publish_road_profile_around_end_point(poses_around_end_point, num_poses_acquired);

	free(poses_around_end_point);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	carmen_point_t robot_pose = msg->globalpos;
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
	carmen_check_for_annotations(robot_pose, carmen_rddf_poses_ahead, carmen_rddf_poses_back,
			carmen_rddf_num_poses_ahead, carmen_rddf_num_poses_back, msg->timestamp);

	carmen_rddf_play_publish_rddf_and_annotations(robot_pose);
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	carmen_point_t robot_pose = msg->truepose;
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
	carmen_check_for_annotations(robot_pose, carmen_rddf_poses_ahead, carmen_rddf_poses_back,
			carmen_rddf_num_poses_ahead, carmen_rddf_num_poses_back, msg->timestamp);

	carmen_rddf_play_publish_rddf_and_annotations(robot_pose);
}


//static void
//carmen_rddf_play_nearest_waypoint_message_handler(carmen_rddf_nearest_waypoint_message *rddf_nearest_waypoint_message)
//{
//	carmen_rddf_nearest_waypoint_to_end_point = rddf_nearest_waypoint_message->point;
//	carmen_rddf_nearest_waypoint_is_set = 1;
//
//	carmen_rddf_publish_nearest_waypoint_confirmation_message(rddf_nearest_waypoint_message->point);
//	already_reached_nearest_waypoint_to_end_point = 0;
//}


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


static void
carmen_traffic_light_message_handler(carmen_traffic_light_message *message)
{
	traffic_lights = message;
}


static void
carmen_rddf_dynamic_annotation_message_handler(carmen_rddf_dynamic_annotation_message *message)
{
	// Avoid reinserting the same annotation over and over.
	for (size_t i = 0; i < dynamic_annotation_messages.size(); i++)
	{
		carmen_rddf_dynamic_annotation_message &stored = dynamic_annotation_messages[i];
		if (stored.annotation_type == message->annotation_type &&
			stored.annotation_code == message->annotation_code &&
			DIST2D(stored.annotation_point, message->annotation_point) < 0.5)
		{
			stored = *message;
			return;
		}
	}

	dynamic_annotation_messages.push_front(*message);
	if (dynamic_annotation_messages.size() > 30)
		dynamic_annotation_messages.pop_back();
}


void
carmen_moving_objects_point_clouds_message_handler(carmen_moving_objects_point_clouds_message *moving_objects_point_clouds_message)
{
	moving_objects = moving_objects_point_clouds_message;
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_rddf_play_subscribe_messages()
{
	if (!use_truepos)
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

//	carmen_rddf_subscribe_nearest_waypoint_message(NULL,
//			(carmen_handler_t) carmen_rddf_play_nearest_waypoint_message_handler,
//			CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_end_point_message(NULL,
			(carmen_handler_t) carmen_rddf_play_end_point_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

    carmen_traffic_light_subscribe(traffic_lights_camera, NULL, (carmen_handler_t) carmen_traffic_light_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_rddf_subscribe_dynamic_annotation_message(NULL, (carmen_handler_t) carmen_rddf_dynamic_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_moving_objects_point_clouds_subscribe_message(NULL, (carmen_handler_t) carmen_moving_objects_point_clouds_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
carmen_rddf_play_load_index(char *rddf_filename)
{
	int annotation = 0;
	carmen_fused_odometry_message message;
	placemark_vector_t placemark_vector;

	if (!carmen_rddf_index_exists(rddf_filename))
	{
		if (strcmp(rddf_filename + (strlen(rddf_filename) - 3), "kml") == 0)
		{
			carmen_rddf_play_open_kml(rddf_filename, &placemark_vector);

			for (unsigned int i = 0; i < placemark_vector.size(); i++)
			{
				if (carmen_rddf_play_copy_kml(placemark_vector[i], &message, &annotation))
					carmen_rddf_index_add(&message, 0, 0, annotation);
			}

			carmen_rddf_index_save(rddf_filename);
		}
		else
		{
			FILE *fptr = fopen(rddf_filename, "r");

			while (!feof(fptr))
			{
				memset(&message, 0, sizeof(message));

				fscanf(fptr, "%lf %lf %lf %lf %lf %lf\n",
					&(message.pose.position.x), &(message.pose.position.y),
					&(message.pose.orientation.yaw), &(message.velocity.x), &(message.phi),
					&(message.timestamp));

				carmen_rddf_index_add(&message, 0, 0, 0);
			}

			carmen_rddf_index_save(rddf_filename);
			fclose(fptr);
		}
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
		carmen_annotation_t annotation;
		annotation.annotation_description = (char *) calloc (1024, sizeof(char));

		fscanf(f, "%s\t%d\t%d\t%lf\t%lf\t%lf\t%lf\n",
			annotation.annotation_description,
			&annotation.annotation_type,
			&annotation.annotation_code,
			&annotation.annotation_orientation,
			&annotation.annotation_point.x,
			&annotation.annotation_point.y,
			&annotation.annotation_point.z
		);
		carmen_ackerman_traj_point_t annotation_point;
		annotation_point.x = annotation.annotation_point.x;
		annotation_point.y = annotation.annotation_point.y;
		annotation_point.theta = annotation.annotation_orientation;
		double distance_car_pose_car_front = distance_between_front_and_rear_axles + distance_between_front_car_and_front_wheels;
		carmen_point_t new_annotation_point = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&annotation_point, -distance_car_pose_car_front);

		annotation.annotation_point.x = new_annotation_point.x;
		annotation.annotation_point.y = new_annotation_point.y;
		annotation_read_from_file.push_back(annotation);
	}

	fclose(f);
}


static void
carmen_rddf_play_get_parameters(int argc, char** argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "robot", (char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &distance_between_front_and_rear_axles, 1, NULL},
		{(char *) "robot", (char *) "distance_between_front_car_and_front_wheels", CARMEN_PARAM_DOUBLE, &distance_between_front_car_and_front_wheels, 1, NULL},
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
	annotations_codes = (int *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(int));
	memset(&annotation_queue_message, 0, sizeof(annotation_queue_message));

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
		exit(printf("Error: Use %s <rddf> <annotations> <traffic_lights_camera>\n", argv[0]));

	carmen_rddf_filename = argv[1];

	if (argc == 2)
	{
		carmen_annotation_filename = NULL;
		traffic_lights_camera = 3;
	}
	else if (argc == 3)
	{
		carmen_annotation_filename = argv[2];
		traffic_lights_camera = 3;
	}
	else if (argc == 4)
	{
		carmen_annotation_filename = argv[2];
		traffic_lights_camera = atoi(argv[3]);
	}

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

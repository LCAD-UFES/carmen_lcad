#include <algorithm>
#include <carmen/carmen.h>
#include <carmen/traffic_light_interface.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/voice_interface_interface.h>

#include "rddf_interface.h"
#include "rddf_index.h"
#include "rddf_util.h"

//#define PRINT_DEBUG

extern bool use_road_map;

static bool robot_pose_queued = false;
carmen_localize_ackerman_globalpos_message *current_globalpos_msg = NULL;
static carmen_simulator_ackerman_truepos_message *current_truepos_msg = NULL;
static carmen_map_p current_road_map = NULL;

extern char *carmen_rddf_filename;

extern int carmen_rddf_num_poses_ahead_max;
extern int rddf_play_use_truepos;

int traffic_sign_is_on = false;
int traffic_sign_code = RDDF_ANNOTATION_CODE_NONE;
double traffic_sign_curvature = 0.0;
int state_traffic_sign_code = RDDF_ANNOTATION_CODE_NONE;
double state_traffic_sign_curvature = 0.0;

static int carmen_rddf_end_point_is_set = 0;
carmen_robot_and_trailer_pose_t carmen_rddf_end_point;

int carmen_rddf_nearest_waypoint_is_set = 0;
carmen_robot_and_trailer_pose_t carmen_rddf_nearest_waypoint_to_end_point;

carmen_robot_and_trailer_traj_point_t *carmen_rddf_poses_ahead = NULL;
static carmen_robot_and_trailer_traj_point_t *carmen_rddf_poses_back = NULL;
int carmen_rddf_num_poses_ahead = 0;
static int carmen_rddf_num_poses_back = 0;
int *annotations_codes;
int *annotations;

static int carmen_rddf_pose_initialized = 0;
int already_reached_nearest_waypoint_to_end_point = 0;

vector<annotation_and_index> annotations_to_publish;
carmen_rddf_annotation_message annotation_queue_message;

extern int traffic_lights_camera;
carmen_traffic_light_message *traffic_lights = NULL;

deque<carmen_rddf_dynamic_annotation_message> dynamic_annotation_messages;

carmen_moving_objects_point_clouds_message *moving_objects = NULL;
carmen_moving_objects_point_clouds_message *pedestrians_tracked = NULL;

bool simulated_pedestrian_on = false;


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

//	printf ("Annotation size %d\n", (int)annotations_to_publish.size());

	for (size_t i = 0; i < annotations_to_publish.size(); i++)
	{
//		printf ("code %d\n", annotations_to_publish[i].annotation.annotation_type);
		memcpy(&(annotation_queue_message.annotations[i]), &(annotations_to_publish[i].annotation), sizeof(carmen_annotation_t));
	}
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
		carmen_rddf_play_clear_annotations();
		carmen_rddf_play_set_annotations(robot_pose);

		carmen_rddf_publish_road_profile_message(
			carmen_rddf_poses_ahead,
			carmen_rddf_poses_back,
			carmen_rddf_num_poses_ahead,
			carmen_rddf_num_poses_back,
			annotations,
			annotations_codes);

		carmen_rddf_play_publish_annotation_queue();

		carmen_rddf_publish_traffic_sign_message(state_traffic_sign_code, state_traffic_sign_curvature);
	}
}


void
carmen_rddf_play_find_and_publish_poses_around_end_point(double x, double y, double yaw, int num_poses_desired, double timestamp)
{
	int num_poses_acquired = 0;
	carmen_robot_and_trailer_traj_point_t *poses_around_end_point;

	poses_around_end_point = (carmen_robot_and_trailer_traj_point_t *) calloc (num_poses_desired, sizeof(carmen_robot_and_trailer_traj_point_t));
	carmen_test_alloc(poses_around_end_point);

	num_poses_acquired = carmen_find_poses_around(x, y, yaw, timestamp, poses_around_end_point, num_poses_desired);
	carmen_rddf_publish_road_profile_around_end_point_message(poses_around_end_point, num_poses_acquired);

	free(poses_around_end_point);
}
///////////////////////////////////////////////////////////////////////////////////////////////


static void
build_and_send_rddf_and_annotations(carmen_point_t robot_pose, double timestamp)
{
	if (use_road_map)
	{
		robot_pose_queued = (current_road_map == NULL || carmen_rddf_play_pose_out_of_map_coordinates(robot_pose, current_road_map));
		if (robot_pose_queued)
			return;
		carmen_rddf_play_check_reset_traffic_sign_state(robot_pose);
		carmen_rddf_num_poses_ahead = carmen_rddf_play_find_nearest_poses_by_road_map(
				robot_pose,
				current_road_map,
				carmen_rddf_poses_ahead,
				carmen_rddf_poses_back,
				&carmen_rddf_num_poses_back,
				carmen_rddf_num_poses_ahead_max);
	}
	else
	{
		carmen_rddf_num_poses_ahead = carmen_rddf_play_find_nearest_poses_ahead(
				robot_pose.x,
				robot_pose.y,
				robot_pose.theta,
				0.0,
				timestamp,
				carmen_rddf_poses_ahead,
				carmen_rddf_poses_back,
				&carmen_rddf_num_poses_back,
				carmen_rddf_num_poses_ahead_max,
				annotations);
	}

	annotations_to_publish.clear();
	carmen_check_for_annotations(robot_pose, carmen_rddf_poses_ahead, carmen_rddf_poses_back,
			carmen_rddf_num_poses_ahead, carmen_rddf_num_poses_back, timestamp);

	carmen_rddf_play_publish_rddf_and_annotations(robot_pose);
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
	carmen_rddf_pose_initialized = 1;
	current_globalpos_msg = msg;
	build_and_send_rddf_and_annotations(msg->globalpos, msg->timestamp);
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	carmen_rddf_pose_initialized = 1;
	current_truepos_msg = msg;
	build_and_send_rddf_and_annotations(msg->truepose, msg->timestamp);
}


static void
road_map_handler(carmen_map_server_road_map_message *msg)
{
	static bool first_time = true;

	if (first_time)
	{
		current_road_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
		carmen_grid_mapping_initialize_map(current_road_map, msg->config.x_size, msg->config.resolution, 'r');
		first_time = false;
	}

	if (msg->config.x_origin != current_road_map->config.x_origin || msg->config.y_origin != current_road_map->config.y_origin) // new map
	{
		memcpy(current_road_map->complete_map, msg->complete_map, sizeof(double) * msg->size);
		current_road_map->config = msg->config;

		if (robot_pose_queued)
		{
			if (rddf_play_use_truepos)
				simulator_ackerman_truepos_message_handler(current_truepos_msg);
			else
				localize_globalpos_handler(current_globalpos_msg);
		}
	}
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
carmen_rddf_update_annotation_message_handler(carmen_rddf_update_annotation_message *message)
{
	carmen_rddf_play_updade_annotation_vector(message->action, message->old_annotation, message->new_annotation);
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


void
carmen_voice_interface_command_message_handler(carmen_voice_interface_command_message *message)
{
	if (message->command_id == SET_COURSE)
	{
		printf("New rddf set by voice command: %s\n", message->command);

		carmen_rddf_index_clear();

		char *carmen_home = getenv("CARMEN_HOME");
		static char rddf_file_name[2048];
		strcpy(rddf_file_name, carmen_home);
		strcat(rddf_file_name, "/");
		strcat(rddf_file_name, message->command);

		carmen_rddf_play_load_index(rddf_file_name);

		char carmen_annotation_filename[2048];
		rddf_file_name[strlen(rddf_file_name) - 4] = '\0';
		sprintf(carmen_annotation_filename, "%s_annotation.txt", rddf_file_name);

		carmen_rddf_play_load_annotation_file(carmen_annotation_filename);
	}
}


static void
carmen_rddf_play_shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		exit(0);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_rddf_play_subscribe_messages()
{
	if (!rddf_play_use_truepos)
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	if (use_road_map)
		carmen_map_server_subscribe_road_map(NULL, (carmen_handler_t) road_map_handler, CARMEN_SUBSCRIBE_LATEST);

//	carmen_rddf_subscribe_nearest_waypoint_message(NULL,
//			(carmen_handler_t) carmen_rddf_play_nearest_waypoint_message_handler,
//			CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_end_point_message(NULL, (carmen_handler_t) carmen_rddf_play_end_point_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_traffic_light_subscribe(traffic_lights_camera, NULL, (carmen_handler_t) carmen_traffic_light_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_update_annotation_message(NULL, (carmen_handler_t) carmen_rddf_update_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_rddf_subscribe_dynamic_annotation_message(NULL, (carmen_handler_t) carmen_rddf_dynamic_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_moving_objects_point_clouds_subscribe_message(NULL, (carmen_handler_t) carmen_moving_objects_point_clouds_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_voice_interface_subscribe_command_message(NULL, (carmen_handler_t) carmen_voice_interface_command_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
carmen_rddf_play_initialize(void)
{
	carmen_rddf_poses_ahead = (carmen_robot_and_trailer_traj_point_t *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(carmen_robot_and_trailer_traj_point_t));
	carmen_rddf_poses_back = (carmen_robot_and_trailer_traj_point_t *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(carmen_robot_and_trailer_traj_point_t));
	annotations = (int *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(int));
	annotations_codes = (int *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(int));
	memset(&annotation_queue_message, 0, sizeof(annotation_queue_message));

	carmen_test_alloc(carmen_rddf_poses_ahead);
	carmen_test_alloc(annotations);
}


int
main(int argc, char **argv)
{
	setlocale(LC_ALL, "C");

	char *carmen_annotation_filename = carmen_rddf_play_parse_input_command_line_parameters(argc, argv);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	carmen_rddf_play_get_parameters(argc, argv);
	carmen_rddf_play_initialize();
	carmen_rddf_define_messages();
	carmen_rddf_play_subscribe_messages();
	signal (SIGINT, carmen_rddf_play_shutdown_module);

	if (!use_road_map)
		carmen_rddf_play_load_index(carmen_rddf_filename);

	carmen_rddf_play_load_annotation_file(carmen_annotation_filename);

	carmen_ipc_dispatch();

	return (0);
}

#include <string.h>
#include <stdlib.h>
#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <prob_interface.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/rotation_geometry.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/stereo_velodyne_interface.h>
#include <prob_map.h>
#include <carmen/grid_mapping.h>
#include <carmen/stereo_mapping_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/ultrasonic_filter_interface.h>
#include "laser_calibration.h"

#include "message_interpolation.cpp"

carmen_map_t offline_map;

static int visual_odometry_is_global_pos = 0;

tf::Transformer tf_transformer(false);
MessageInterpolation<carmen_localize_ackerman_globalpos_message, carmen_ultrasonic_sonar_sensor_message> interpolator(1);

static carmen_pose_3D_t car_pose_g;
static carmen_pose_3D_t camera_pose_g;
static carmen_pose_3D_t sensor_board_pose_g;
static carmen_localize_ackerman_map_t localize_map;

/**
 * Model params
 */

double safe_range_above_sensors;
double robot_wheel_radius;

int use_simulator_pose = 0;

rotation_matrix *board_to_car_matrix = NULL;

double highest_sensor = 0.0;

int merge_with_offline_map;
int build_snapshot_map;
int remission_calibration_using_a_map;
int update_and_merge_with_mapper_saved_maps;

carmen_pose_3D_t sensor_board_1_pose;

sensor_parameters_t sensors_params;
sensor_data_t sensors_data;

carmen_pose_3D_t velodyne_pose;

char *map_path;


/*********************************************************
		   --- Handlers ---
 **********************************************************/

static void
offline_map_handler(carmen_map_server_offline_map_message *msg)
{
	carmen_position_t map_origin;

	map_origin.x = msg->config.x_origin;
	map_origin.y = msg->config.y_origin;

	laser_calibration(&map_origin);
}


static void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	if (visual_odometry_is_global_pos)
		interpolator.AddMessageToInterpolationList(globalpos_message);
	else
		mapper_set_robot_pose_into_the_map(globalpos_message);

}


static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	mapper_velodyne_partial_scan(velodyne_message);

}


static void
localize_map_update_handler(carmen_map_server_localize_map_message* message)
{
	carmen_map_server_localize_map_message_to_localize_map(message, &localize_map);

	laser_calibration_using_map(&localize_map);

}



static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		if (update_and_merge_with_mapper_saved_maps)
			mapper_save_current_map();
			
		carmen_ipc_disconnect();
		fprintf(stderr, "Shutdown laser_calibration_main\n");

		exit(0);
	}
}


/*********************************************************
	   --- Initialization functions ---
 **********************************************************/


static void
init_velodyne_points(spherical_point_cloud **velodyne_points_out, unsigned char ***intencity, carmen_pose_3D_t **robot_pose_out, carmen_vector_3D_t **robot_velocity_out, double **robot_timestamp_out, double **robot_phi_out)
{
	int i;

	carmen_pose_3D_t *robot_pose = (carmen_pose_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_pose_3D_t));
	carmen_vector_3D_t *robot_velocity = (carmen_vector_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_vector_3D_t));
	spherical_point_cloud *velodyne_points = (spherical_point_cloud *)malloc(NUM_VELODYNE_POINT_CLOUDS * sizeof(spherical_point_cloud));
	double *robot_timestamp = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));
	*intencity = (unsigned char **)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(unsigned char*));
	*robot_phi_out = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));


	carmen_test_alloc(velodyne_points);

	for (i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
	{
		velodyne_points[i].num_points = 0;
		velodyne_points[i].sphere_points = NULL;
	}

	*velodyne_points_out = velodyne_points;
	*robot_pose_out = robot_pose;
	*robot_velocity_out = robot_velocity;
	*robot_timestamp_out = robot_timestamp;
}


static void
get_alive_sensors(int argc, char **argv)
{
	carmen_param_t param_list[] =
	{
			{(char*)"mapper", (char*)"velodyne", CARMEN_PARAM_ONOFF, &sensors_params.alive, 0, NULL},
			{(char*)"mapper", (char*)"velodyne_locc", CARMEN_PARAM_DOUBLE, &sensors_params.log_odds.log_odds_occ, 0, NULL},
			{(char*)"mapper", (char*)"velodyne_lfree", CARMEN_PARAM_DOUBLE, &sensors_params.log_odds.log_odds_free, 0, NULL},
			{(char*)"mapper", (char*)"velodyne_l0", CARMEN_PARAM_DOUBLE, &sensors_params.log_odds.log_odds_l0, 0, NULL},
			{(char*)"mapper", (char*)"velodyne_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params.unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"mapper", (char*)"velodyne_lambda_short_min", CARMEN_PARAM_DOUBLE, &sensors_params.lambda_short_min, 0, NULL},
			{(char*)"mapper", (char*)"velodyne_lambda_short_max", CARMEN_PARAM_DOUBLE, &sensors_params.lambda_short_max, 0, NULL},
			{(char*)"mapper",  (char*)"velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &sensors_params.range_max_factor, 0, NULL}

	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}

static void
get_sensors_param(int argc, char **argv)
{

	int i;

	sensors_params.pose = velodyne_pose;
	sensors_params.sensor_robot_reference = carmen_change_sensor_reference(sensor_board_1_pose.position, sensors_params.pose.position, board_to_car_matrix);

	sensors_params.height = sensors_params.sensor_robot_reference.z + robot_wheel_radius;

	if (sensors_params.height > highest_sensor)
		highest_sensor = sensors_params.height;

	if (sensors_params.alive)
	{
		sensors_params.ray_order = carmen_velodyne_get_ray_order();
		sensors_params.vertical_correction = carmen_velodyne_get_vertical_correction();
		sensors_params.delta_difference_mean = carmen_velodyne_get_delta_difference_mean();
		sensors_params.delta_difference_stddev = carmen_velodyne_get_delta_difference_stddev();

		carmen_param_t param_list[] =
		{
				{(char *)"velodyne", (char*)"vertical_resolution", CARMEN_PARAM_INT, &sensors_params.vertical_resolution, 0, NULL},
				{(char *)"mapper", (char*)"velodyne_range_max", CARMEN_PARAM_DOUBLE, &sensors_params.range_max, 0, NULL},
				{(char *)"velodyne", (char*)"time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &sensors_params.time_spent_by_each_scan, 0, NULL},

		};

		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		init_velodyne_points(&sensors_data.points, &sensors_data.intensity, &sensors_data.robot_pose, &sensors_data.robot_velocity, &sensors_data.robot_timestamp, &sensors_data.robot_phi);
		sensors_params.sensor_to_support_matrix = create_rotation_matrix(sensors_params.pose.orientation);
		sensors_data.point_cloud_index = 0;
		carmen_prob_models_alloc_sensor_data(&sensors_data, sensors_params.vertical_resolution);

		if (remission_calibration_using_a_map)
		{
			sensors_params.remission_calibration = (double *) calloc(256 * sensors_params.vertical_resolution, sizeof(double));
			FILE *f = fopen("remission_calibration.txt", "r");
			for (i = 0; i < 256 * sensors_params.vertical_resolution; i++)
			{
				fscanf(f, "%lf", &sensors_params.remission_calibration[i]);
			}
			fclose(f);
		}

	}

}


/* read all parameters from .ini file and command line. */
static void
read_parameters(int argc, char **argv,
		carmen_map_config_t *map_config,
		carmen_robot_ackerman_config_t *p_car_config)
{
	double robot_vertical_displacement_from_center;

	double map_resolution, map_width, map_height;
	carmen_param_t param_list[] =
	{
			{(char*)"robot",  (char*)"distance_between_front_car_and_front_wheels", 	CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_front_car_and_front_wheels), 1, NULL},
			{(char*)"robot",  (char*)"distance_between_front_and_rear_axles",		CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_front_and_rear_axles), 1, NULL},
			{(char*)"robot",  (char*)"distance_between_rear_car_and_rear_wheels",		CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_rear_car_and_rear_wheels), 1, NULL},
			{(char*)"robot",  (char*)"distance_between_rear_wheels",			CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_rear_wheels), 1, NULL},

			{(char*)"sensor_board_1",  (char*)"x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x),	0, NULL},
			{(char*)"sensor_board_1",  (char*)"y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y),	0, NULL},
			{(char*)"sensor_board_1",  (char*)"z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z),	0, NULL},
			{(char*)"sensor_board_1",  (char*)"roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll),0, NULL},
			{(char*)"sensor_board_1",  (char*)"pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch),0, NULL},
			{(char*)"sensor_board_1",  (char*)"yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw),	0, NULL},

			{(char*)"velodyne",  (char*)"x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
			{(char*)"velodyne",  (char*)"y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
			{(char*)"velodyne",  (char*)"z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
			{(char*)"velodyne",  (char*)"roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
			{(char*)"velodyne",  (char*)"pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
			{(char*)"velodyne",  (char*)"yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

			{(char*)"robot",  (char*)"wheel_radius", CARMEN_PARAM_DOUBLE, &(robot_wheel_radius), 0, NULL},
			{(char*)"mapper",  (char*)"safe_range_above_sensors", CARMEN_PARAM_DOUBLE, &safe_range_above_sensors, 0, NULL},

			{(char*)"robot",  (char*)"length", CARMEN_PARAM_DOUBLE, &p_car_config->length, 0, NULL},
			{(char*)"robot",  (char*)"width", CARMEN_PARAM_DOUBLE, &p_car_config->width, 0, NULL},
			{(char*)"robot",  (char*)"vertical_displacement_from_center", CARMEN_PARAM_DOUBLE, &robot_vertical_displacement_from_center, 0, NULL},

			{(char*)"mapper",  (char*)"map_grid_res", CARMEN_PARAM_DOUBLE, &map_resolution, 0, NULL},
			{(char*)"mapper",  (char*)"map_width", CARMEN_PARAM_DOUBLE, &map_width, 0, NULL},
			{(char*)"mapper",  (char*)"map_height", CARMEN_PARAM_DOUBLE, &map_height, 0, NULL},


			{(char*)"mapper",  (char*)"build_snapshot_map", CARMEN_PARAM_ONOFF, &build_snapshot_map, 0, NULL},
			{(char*)"mapper",  (char*)"merge_with_offline_map", CARMEN_PARAM_ONOFF, &merge_with_offline_map, 0, NULL},
			{(char*)"mapper",  (char*)"update_and_merge_with_mapper_saved_maps", CARMEN_PARAM_ONOFF, &update_and_merge_with_mapper_saved_maps, 0, NULL},

			{(char *) "visual_odometry", (char *) "is_global_pos", CARMEN_PARAM_ONOFF, &visual_odometry_is_global_pos, 0, NULL},
			{(char*) "laser_calibration",  (char*)"remission_calibration_using_a_map", CARMEN_PARAM_ONOFF, &remission_calibration_using_a_map, 0, NULL},

	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));


	if (map_width != map_height)
		carmen_die("Wrong map size: width (%f) must be equal to height (%f).", map_width, map_height);

	if ( (((int)map_width) % 3) != 0)
		carmen_die("Wrong map size: width (%f) and height (%f) must be multiple of 3.", map_width, map_height);

	map_config->x_size = round(map_width / map_resolution);
	map_config->y_size = round(map_height / map_resolution);
	map_config->resolution = map_resolution;

	carmen_grid_mapping_init_parameters(map_resolution, map_width);

	board_to_car_matrix = create_rotation_matrix(sensor_board_1_pose.orientation);

	get_alive_sensors(argc, argv);

	get_sensors_param(argc, argv);
}

static void
subscribe_to_sensor_messages()
{

	carmen_localize_ackerman_subscribe_globalpos_message(NULL,
			(carmen_handler_t)carmen_localize_ackerman_globalpos_message_handler,
			CARMEN_SUBSCRIBE_LATEST);
	if (sensors_params.alive)
	{
		carmen_velodyne_subscribe_partial_scan_message(NULL,
				(carmen_handler_t)velodyne_partial_scan_message_handler,
				CARMEN_SUBSCRIBE_LATEST);
	}

	if (remission_calibration_using_a_map)
		carmen_map_server_subscribe_localize_map_message(NULL,
				(carmen_handler_t) localize_map_update_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) offline_map_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
define_mapper_messages()
{
	/* register initialize message */
	carmen_grid_mapping_define_messages();
}

static void
init_localize_map()
{
	localize_map.carmen_map.complete_map = NULL;
	localize_map.complete_distance = NULL;
	localize_map.complete_gprob = NULL;
	localize_map.complete_prob = NULL;
	localize_map.complete_x_offset = NULL;
	localize_map.complete_y_offset = NULL;

	localize_map.carmen_map.map = NULL;
	localize_map.distance = NULL;
	localize_map.gprob = NULL;
	localize_map.prob = NULL;
	localize_map.x_offset = NULL;
	localize_map.y_offset = NULL;
}


void initialize_transforms()
{
	tf::Transform board_to_camera_pose;
	tf::Transform car_to_board_pose;
	tf::Transform world_to_car_pose;
	tf::Transform ultrasonic_sensor_r1_to_car_pose;
	tf::Transform ultrasonic_sensor_r2_to_car_pose;
	tf::Transform ultrasonic_sensor_l1_to_car_pose;
	tf::Transform ultrasonic_sensor_l2_to_car_pose;

	tf::Time::init();

	// initial car pose with respect to the world
	world_to_car_pose.setOrigin(tf::Vector3(car_pose_g.position.x, car_pose_g.position.y, car_pose_g.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(car_pose_g.orientation.yaw, car_pose_g.orientation.pitch, car_pose_g.orientation.roll));
	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	tf_transformer.setTransform(world_to_car_transform, "world_to_car_transform");

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(sensor_board_pose_g.position.x, sensor_board_pose_g.position.y, sensor_board_pose_g.position.z));
	car_to_board_pose.setRotation(tf::Quaternion(sensor_board_pose_g.orientation.yaw, sensor_board_pose_g.orientation.pitch, sensor_board_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	tf_transformer.setTransform(car_to_board_transform, "car_to_board_transform");

	// camera pose with respect to the board
	board_to_camera_pose.setOrigin(tf::Vector3(camera_pose_g.position.x, camera_pose_g.position.y, camera_pose_g.position.z));
	board_to_camera_pose.setRotation(tf::Quaternion(camera_pose_g.orientation.yaw, camera_pose_g.orientation.pitch, camera_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform board_to_camera_transform(board_to_camera_pose, tf::Time(0), "/board", "/camera");
	tf_transformer.setTransform(board_to_camera_transform, "board_to_camera_transform");

}


int
main(int argc, char **argv)
{
	carmen_map_config_t map_config;
	carmen_robot_ackerman_config_t car_config;

	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Initialize all the relevant parameters */
	read_parameters(argc, argv, &map_config, &car_config);

	initialize_transforms();

	init_localize_map();

	mapper_initialize(&map_config, car_config, sensors_params.vertical_resolution);

	/* Register messages */
	define_mapper_messages();

	/* Subscribe to sensor messages */
	subscribe_to_sensor_messages();


	carmen_ipc_dispatch();

	return (0);
}

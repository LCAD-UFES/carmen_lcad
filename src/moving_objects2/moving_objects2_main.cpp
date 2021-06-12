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
#include <carmen/mapper_interface.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/stereo_velodyne_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/stereo_mapping_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/ultrasonic_filter_interface.h>
#include <carmen/parking_assistant_interface.h>
#include <omp.h>
#include "moving_objects2.h"


carmen_map_t offline_map;
// TODO: essa variavel eh definida como externa dentro da lib do mapper. Corrigir!
carmen_localize_ackerman_globalpos_message *globalpos_history;
int last_globalpos;


/**
 * Model params
 */

double safe_range_above_sensors;
double robot_wheel_radius;

double obstacle_probability_threshold = 0.5;

int use_simulator_pose = 0;

rotation_matrix *board_to_car_matrix = NULL;

double highest_sensor = 0.0;

int merge_with_offline_map;
int build_snapshot_map;
int update_cells_below_car;
int update_and_merge_with_moving_objects2_saved_maps;
int update_and_merge_with_snapshot_map;
int decay_to_offline_map;
int create_map_sum_and_count;


carmen_pose_3D_t sensor_board_1_pose;

sensor_parameters_t *sensors_params;
sensor_parameters_t ultrasonic_sensor_params;
sensor_data_t *sensors_data;
int number_of_sensors;

carmen_pose_3D_t velodyne_pose;

char *map_path;

int publish_moving_objects_raw_map;

carmen_rddf_annotation_message last_rddf_annotation_message;
int robot_near_strong_slow_down_annotation = 0;

bool offline_map_available = false;
int ok_to_publish = 0;
int number_of_threads = 1;

rotation_matrix *r_matrix_car_to_global = NULL;


carmen_localize_ackerman_map_t localize_map;

cv::Mat road_map;

double x_origin, y_origin;



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
localize_map_update_handler(carmen_map_server_localize_map_message *message)
{
	static bool first_time = true;

	if (first_time)
	{
		offline_map_available = true;
		first_time = false;
	}

	carmen_map_server_localize_map_message_to_localize_map(message, &localize_map);

	x_origin = message->config.x_origin;
	y_origin = message->config.y_origin;

	road_map = segment_remission_map(&localize_map.carmen_mean_remission_map, &localize_map.carmen_map);
}


static void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{

	moving_objects2_set_robot_pose_into_the_map(globalpos_message, update_cells_below_car);


	if (ok_to_publish)
	{
		int aux = -1;
		for (int i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
		{
			if (sensors_data[0].points_timestamp[i] == globalpos_message->timestamp)
			{
				aux = sensors_data[0].point_cloud_index;
				sensors_data[0].point_cloud_index = i;
				run_moving_objects2(&sensors_params[0], &sensors_data[0], r_matrix_car_to_global);
				sensors_data[0].point_cloud_index = aux;
				break;
			}
		}
	}
}

static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	moving_objects2_velodyne_partial_scan(velodyne_message);
}


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		fprintf(stderr, "Shutdown moving_objects2_main\n");

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
init_velodyne_points(spherical_point_cloud **velodyne_points_out, unsigned char ***intencity, carmen_pose_3D_t **robot_pose_out,
		carmen_vector_3D_t **robot_velocity_out, double **robot_timestamp_out, double **robot_phi_out, double **points_timestamp_out)
{
	int i;

	carmen_pose_3D_t *robot_pose = (carmen_pose_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_pose_3D_t));
	carmen_vector_3D_t *robot_velocity = (carmen_vector_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_vector_3D_t));
	spherical_point_cloud *velodyne_points = (spherical_point_cloud *)malloc(NUM_VELODYNE_POINT_CLOUDS * sizeof(spherical_point_cloud));
	double *robot_timestamp = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));
	*intencity = (unsigned char **)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(unsigned char*));
	*robot_phi_out = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));
	*points_timestamp_out = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));


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
	int i;

	sensors_params = (sensor_parameters_t *)calloc(number_of_sensors, sizeof(sensor_parameters_t));
	carmen_test_alloc(sensors_params);

	sensors_data = (sensor_data_t *)calloc(number_of_sensors, sizeof(sensor_data_t));
	carmen_test_alloc(sensors_data);

	carmen_param_t param_list[] =
	{
			{(char *) "mapper", (char *) "velodyne", CARMEN_PARAM_ONOFF, &sensors_params[0].alive, 0, NULL},
			{(char *) "mapper", (char *) "velodyne_locc", CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_occ, 0, NULL},
			{(char *) "mapper", (char *) "velodyne_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_free, 0, NULL},
			{(char *) "mapper", (char *) "velodyne_l0", CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_l0, 0, NULL},
			{(char *) "mapper", (char *) "velodyne_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[0].unexpeted_delta_range_sigma, 0, NULL},

			{(char *) "mapper", (char *) "unsafe_height_above_ground", CARMEN_PARAM_DOUBLE, &sensors_params[0].unsafe_height_above_ground, 0, NULL},

			{(char *) "mapper",  (char *) "velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &sensors_params[0].range_max_factor, 0, NULL}


	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	for (i = 0; i < number_of_sensors; i++)
	{
		if (i == STEREO_MAPPING_SENSOR_INDEX)
			continue;

		sensors_params[i].unsafe_height_above_ground = sensors_params[0].unsafe_height_above_ground;

		sensors_data[i].ray_position_in_the_floor = (carmen_vector_2D_t**)calloc(number_of_threads ,sizeof(carmen_vector_2D_t*));
		sensors_data[i].maxed = (int**)calloc(number_of_threads ,sizeof(int*));
		sensors_data[i].obstacle_height = (double**)calloc(number_of_threads ,sizeof(double*));
		sensors_data[i].occupancy_log_odds_of_each_ray_target = (double**)calloc(number_of_threads ,sizeof(double*));
		sensors_data[i].point_cloud_index = 0;
		sensors_data[i].points = NULL;
		sensors_data[i].ray_origin_in_the_floor = (carmen_vector_2D_t**)calloc(number_of_threads ,sizeof(carmen_vector_2D_t*));;
		sensors_data[i].ray_size_in_the_floor = (double**)calloc(number_of_threads ,sizeof(double*));
		sensors_data[i].processed_intensity = (double**)calloc(number_of_threads ,sizeof(double*));
		sensors_data[i].ray_hit_the_robot = (int**)calloc(number_of_threads ,sizeof(int*));
		sensors_data[i].ray_that_hit_the_nearest_target = (int*)calloc(number_of_threads ,sizeof(int));

		sensors_params[i].name = NULL;
		sensors_params[i].ray_order = NULL;
		sensors_params[i].sensor_to_support_matrix = NULL;
		sensors_params[i].vertical_correction = NULL;
		sensors_params[i].vertical_resolution = 0;

		for (int j = 0; j < number_of_threads; j++)
		{
			sensors_data[i].ray_position_in_the_floor[j] = NULL;
			sensors_data[i].maxed[j] = NULL;
			sensors_data[i].obstacle_height[j] = NULL;
			sensors_data[i].occupancy_log_odds_of_each_ray_target[j] = NULL;
			sensors_data[i].ray_origin_in_the_floor[j] = NULL;
			sensors_data[i].ray_size_in_the_floor[j] = NULL;
			sensors_data[i].processed_intensity[j] = NULL;
			sensors_data[i].ray_hit_the_robot[j] = NULL;
		}

		if (sensors_params[i].alive)
		{
			sensors_params[i].name = (char *)calloc(strlen(param_list[i].variable) + 1, sizeof(char));
			strcpy(sensors_params[i].name, param_list[i].variable);
		}
	}
}


static void
get_sensors_param(int argc, char **argv)
{

	sensors_params[0].pose = velodyne_pose;
	sensors_params[0].sensor_robot_reference = carmen_change_sensor_reference(sensor_board_1_pose.position, sensors_params[0].pose.position, board_to_car_matrix);

	sensors_params[0].height = sensors_params[0].sensor_robot_reference.z + robot_wheel_radius;

	if (sensors_params[0].height > highest_sensor)
		highest_sensor = sensors_params[0].height;

	if (sensors_params[0].alive && !strcmp(sensors_params[0].name,"velodyne"))
	{
		sensors_params[0].ray_order = carmen_velodyne_get_ray_order();
		sensors_params[0].vertical_correction = carmen_velodyne_get_vertical_correction();
		sensors_params[0].delta_difference_mean = carmen_velodyne_get_delta_difference_mean();
		sensors_params[0].delta_difference_stddev = carmen_velodyne_get_delta_difference_stddev();

		carmen_param_t param_list[] =
		{
				{sensors_params[0].name, (char *) "vertical_resolution", CARMEN_PARAM_INT, &sensors_params[0].vertical_resolution, 0, NULL},
				{(char *)"mapper", (char *) "velodyne_range_max", CARMEN_PARAM_DOUBLE, &sensors_params[0].range_max, 0, NULL},
				{sensors_params[0].name, (char *) "time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &sensors_params[0].time_spent_by_each_scan, 0, NULL},

		};

		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		init_velodyne_points(&sensors_data[0].points, &sensors_data[0].intensity, &sensors_data[0].robot_pose, &sensors_data[0].robot_velocity, &sensors_data[0].robot_timestamp, &sensors_data[0].robot_phi, &sensors_data[0].points_timestamp);
		sensors_params[0].sensor_to_support_matrix = create_rotation_matrix(sensors_params[0].pose.orientation);
		sensors_data[0].point_cloud_index = 0;
		carmen_prob_models_alloc_sensor_data(&sensors_data[0], sensors_params[0].vertical_resolution, number_of_threads);

		sensors_params[0].remission_calibration = NULL;//(double *) calloc(256 * sensors_params[0].vertical_resolution, sizeof(double));

	}
	sensors_params[0].current_range_max = sensors_params[0].range_max;

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
			{(char *) "robot",  (char *) "distance_between_front_car_and_front_wheels", CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_front_car_and_front_wheels), 1, NULL},
			{(char *) "robot",  (char *) "distance_between_front_and_rear_axles",		CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_front_and_rear_axles), 1, NULL},
			{(char *) "robot",  (char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_rear_car_and_rear_wheels), 1, NULL},
			{(char *) "robot",  (char *) "distance_between_rear_wheels",				CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_rear_wheels), 1, NULL},

			{(char *) "model", 		(char *) "predictive_planner_obstacles_safe_distance", 	CARMEN_PARAM_DOUBLE, &(p_car_config->model_predictive_planner_obstacles_safe_distance), 1, NULL},
			{(char *) "obstacle", 	(char *) "avoider_obstacles_safe_distance", 			CARMEN_PARAM_DOUBLE, &(p_car_config->obstacle_avoider_obstacles_safe_distance), 1, NULL},

			{(char *) "sensor_board_1",  (char *) "x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x),	0, NULL},
			{(char *) "sensor_board_1",  (char *) "y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y),	0, NULL},
			{(char *) "sensor_board_1",  (char *) "z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z),	0, NULL},
			{(char *) "sensor_board_1",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll),0, NULL},
			{(char *) "sensor_board_1",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch),0, NULL},
			{(char *) "sensor_board_1",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw),	0, NULL},

			{(char *) "velodyne",  (char *) "x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
			{(char *) "velodyne",  (char *) "y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
			{(char *) "velodyne",  (char *) "z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
			{(char *) "velodyne",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
			{(char *) "velodyne",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
			{(char *) "velodyne",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

			{(char *) "robot",  (char *) "wheel_radius", CARMEN_PARAM_DOUBLE, &(robot_wheel_radius), 0, NULL},
			{(char *) "mapper",  (char *) "number_of_sensors", CARMEN_PARAM_INT, &number_of_sensors, 0, NULL},
			{(char *) "mapper",  (char *) "safe_range_above_sensors", CARMEN_PARAM_DOUBLE, &safe_range_above_sensors, 0, NULL},

			{(char *) "robot",  (char *) "length", CARMEN_PARAM_DOUBLE, &p_car_config->length, 0, NULL},
			{(char *) "robot",  (char *) "width", CARMEN_PARAM_DOUBLE, &p_car_config->width, 0, NULL},
			{(char *) "robot",  (char *) "vertical_displacement_from_center", CARMEN_PARAM_DOUBLE, &robot_vertical_displacement_from_center, 0, NULL},

			{(char *) "mapper",  (char *) "map_grid_res", CARMEN_PARAM_DOUBLE, &map_resolution, 0, NULL},
			{(char *) "mapper",  (char *) "map_width", CARMEN_PARAM_DOUBLE, &map_width, 0, NULL},
			{(char *) "mapper",  (char *) "map_height", CARMEN_PARAM_DOUBLE, &map_height, 0, NULL},

			{(char *) "mapper", (char *) "moving_objects_raw_map", CARMEN_PARAM_ONOFF, &publish_moving_objects_raw_map, 0, NULL},

			{(char *) "mapper",  (char *) "build_snapshot_map", CARMEN_PARAM_ONOFF, &build_snapshot_map, 0, NULL},
			{(char *) "mapper",  (char *) "merge_with_offline_map", CARMEN_PARAM_ONOFF, &merge_with_offline_map, 0, NULL},
			{(char *) "mapper",  (char *) "update_and_merge_with_mapper_saved_maps", CARMEN_PARAM_ONOFF, &update_and_merge_with_moving_objects2_saved_maps, 0, NULL},
			{(char *) "mapper",  (char *) "update_cells_below_car", CARMEN_PARAM_ONOFF, &update_cells_below_car, 0, NULL},
			{(char *) "mapper",  (char *) "decay_to_offline_map", CARMEN_PARAM_ONOFF, &decay_to_offline_map, 0, NULL},
			{(char *) "mapper",  (char *) "create_map_sum_and_count", CARMEN_PARAM_ONOFF, &create_map_sum_and_count, 0, NULL},

			{(char *) "mapper",  (char *) "update_and_merge_with_snapshot_map", CARMEN_PARAM_ONOFF, &update_and_merge_with_snapshot_map, 0, NULL},
			{(char *) "mapper",  (char *) "number_of_threads", CARMEN_PARAM_INT, &number_of_threads, 0, NULL}



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
subscribe_to_ipc_messages()
{
	carmen_map_server_subscribe_localize_map_message(NULL,
			(carmen_handler_t) localize_map_update_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_globalpos_message(NULL,
			(carmen_handler_t)carmen_localize_ackerman_globalpos_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[0].alive)
	{
		carmen_velodyne_subscribe_partial_scan_message(NULL,
				(carmen_handler_t)velodyne_partial_scan_message_handler,
				CARMEN_SUBSCRIBE_LATEST);
	}


	// esse handler eh subscribe_all porque todas as anotacoes precisam ser recebidas!
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

//////////////////////////////////////////////////////////////////////////////////////////////////


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

	init_localize_map();

	/* Initialize all the relevant parameters */
	read_parameters(argc, argv, &map_config, &car_config);

	moving_objects2_initialize(&map_config, car_config);

	/* Subscribe to relevant messages */
	subscribe_to_ipc_messages();

	carmen_ipc_dispatch();

	return (0);
}

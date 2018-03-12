/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include <prob_map.h>

#include "velodyne_odometry_velodyne.h"


double safe_range_above_sensors;

#define FUSED_ODOMETRY_VECTOR_SIZE 50

/* global variables */
carmen_pose_3D_t sensor_board_1_pose;
rotation_matrix *sensor_board_1_to_car_matrix;

double robot_wheel_radius;
carmen_robot_ackerman_config_t car_config;
double highest_sensor;

sensor_parameters_t *spherical_sensor_params;
sensor_data_t *spherical_sensor_data;

int number_of_sensors = 1;
double max_range = 0.0;
carmen_pose_3D_t velodyne_pose;

int number_of_threads = 1;

char *calibration_file = NULL;


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	velodyne_odometry_compute_odometry(velodyne_message, &spherical_sensor_params[0], &spherical_sensor_data[0], 0.0, 0.0);
}


static void
shutdown_module(int x)
{
	if (x == SIGINT)
	{
		carmen_verbose("Disconnecting from IPC network.\n");
		exit(1);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


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


void
get_alive_sensors(int argc, char **argv)
{
	int i = 0;

	spherical_sensor_params = (sensor_parameters_t *) calloc(number_of_sensors, sizeof(sensor_parameters_t));
	carmen_test_alloc(spherical_sensor_params);

	spherical_sensor_data = (sensor_data_t *) calloc(number_of_sensors, sizeof(sensor_data_t));
	carmen_test_alloc(spherical_sensor_data);

	carmen_param_t param_list[] =
	{
		{(char *) "localize_ackerman", (char *) "velodyne", CARMEN_PARAM_ONOFF, &spherical_sensor_params[0].alive, 0, NULL},
		{(char *) "localize_ackerman", (char *) "velodyne_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_occ, 0, NULL},
		{(char *) "localize_ackerman", (char *) "velodyne_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_free, 0, NULL},
		{(char *) "localize_ackerman", (char *) "velodyne_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_l0, 0, NULL},
		{(char *) "localize_ackerman", (char *) "velodyne_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].unexpeted_delta_range_sigma, 0, NULL},

		{(char *) "localize_ackerman", (char *) "unsafe_height_above_ground", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].unsafe_height_above_ground, 0, NULL},

		{(char *) "localize_ackerman",  (char *) "velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].range_max_factor, 0, NULL}
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	spherical_sensor_params[i].unsafe_height_above_ground = spherical_sensor_params[0].unsafe_height_above_ground;

	spherical_sensor_data[i].ray_position_in_the_floor = (carmen_vector_2D_t**)calloc(number_of_threads ,sizeof(carmen_vector_2D_t*));
	spherical_sensor_data[i].maxed = (int**)calloc(number_of_threads ,sizeof(int*));
	spherical_sensor_data[i].obstacle_height = (double**)calloc(number_of_threads ,sizeof(double*));
	spherical_sensor_data[i].occupancy_log_odds_of_each_ray_target = (double**)calloc(number_of_threads ,sizeof(double*));
	spherical_sensor_data[i].point_cloud_index = 0;
	spherical_sensor_data[i].points = NULL;
	spherical_sensor_data[i].ray_origin_in_the_floor = (carmen_vector_2D_t**)calloc(number_of_threads ,sizeof(carmen_vector_2D_t*));;
	spherical_sensor_data[i].ray_size_in_the_floor = (double**)calloc(number_of_threads ,sizeof(double*));
	spherical_sensor_data[i].processed_intensity = (double**)calloc(number_of_threads ,sizeof(double*));
	spherical_sensor_data[i].ray_hit_the_robot = (int**)calloc(number_of_threads ,sizeof(int*));
	spherical_sensor_data[i].ray_that_hit_the_nearest_target = (int*)calloc(number_of_threads ,sizeof(int));

	spherical_sensor_params[i].name = NULL;
	spherical_sensor_params[i].ray_order = NULL;
	spherical_sensor_params[i].sensor_to_support_matrix = NULL;
	spherical_sensor_params[i].vertical_correction = NULL;
	spherical_sensor_params[i].vertical_resolution = 0;

	for (int j = 0; j < number_of_threads; j++)
	{
		spherical_sensor_data[i].ray_position_in_the_floor[j] = NULL;
		spherical_sensor_data[i].maxed[j] = NULL;
		spherical_sensor_data[i].obstacle_height[j] = NULL;
		spherical_sensor_data[i].occupancy_log_odds_of_each_ray_target[j] = NULL;
		spherical_sensor_data[i].ray_origin_in_the_floor[j] = NULL;
		spherical_sensor_data[i].ray_size_in_the_floor[j] = NULL;
		spherical_sensor_data[i].processed_intensity[j] = NULL;
		spherical_sensor_data[i].ray_hit_the_robot[j] = NULL;
	}

	if (spherical_sensor_params[i].alive)
	{
		spherical_sensor_params[i].name = (char *)calloc(strlen(param_list[i].variable) + 1, sizeof(char));
		strcpy(spherical_sensor_params[i].name, param_list[i].variable);
	}
}


static void
get_sensors_param(int argc, char **argv)
{
	spherical_sensor_params[0].use_remission = 1;

	if (calibration_file)
		spherical_sensor_params[0].calibration_table = load_calibration_table(calibration_file);
	else
		spherical_sensor_params[0].calibration_table = load_calibration_table((char *) "calibration_table.txt");
	spherical_sensor_params[0].save_calibration_file = NULL;

	spherical_sensor_params[0].pose = velodyne_pose;
	spherical_sensor_params[0].sensor_robot_reference = carmen_change_sensor_reference(sensor_board_1_pose.position, spherical_sensor_params[0].pose.position, sensor_board_1_to_car_matrix);

	spherical_sensor_params[0].height = spherical_sensor_params[0].sensor_robot_reference.z + robot_wheel_radius;

	if (spherical_sensor_params[0].height > highest_sensor)
		highest_sensor = spherical_sensor_params[0].height;

	if (spherical_sensor_params[0].alive && !strcmp(spherical_sensor_params[0].name,"velodyne"))
	{
		spherical_sensor_params[0].ray_order = carmen_velodyne_get_ray_order();
		spherical_sensor_params[0].vertical_correction = carmen_velodyne_get_vertical_correction();
		spherical_sensor_params[0].delta_difference_mean = carmen_velodyne_get_delta_difference_mean();
		spherical_sensor_params[0].delta_difference_stddev = carmen_velodyne_get_delta_difference_stddev();

		carmen_param_t param_list[] =
		{
			{spherical_sensor_params[0].name, 	(char *) "vertical_resolution", 		CARMEN_PARAM_INT, &spherical_sensor_params[0].vertical_resolution, 0, NULL},
			{(char *) "localize_ackerman", 		(char *) "velodyne_range_max", 		CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].range_max, 0, NULL},
			{spherical_sensor_params[0].name, 	(char *) "time_spent_by_each_scan", 	CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].time_spent_by_each_scan, 0, NULL},
		};

		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		init_velodyne_points(&spherical_sensor_data[0].points, &spherical_sensor_data[0].intensity, &spherical_sensor_data[0].robot_pose,
				&spherical_sensor_data[0].robot_velocity, &spherical_sensor_data[0].robot_timestamp, &spherical_sensor_data[0].robot_phi);
		spherical_sensor_params[0].sensor_to_support_matrix = create_rotation_matrix(spherical_sensor_params[0].pose.orientation);
		spherical_sensor_data[0].point_cloud_index = 0;
		carmen_prob_models_alloc_sensor_data(&spherical_sensor_data[0], spherical_sensor_params[0].vertical_resolution, number_of_threads);

		if (max_range < spherical_sensor_params[0].range_max)
			max_range = spherical_sensor_params[0].range_max;

		spherical_sensor_params[0].current_range_max = spherical_sensor_params[0].range_max;
	}
}


static void 
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = 
	{
		{(char *) "robot", (char *) "length", CARMEN_PARAM_DOUBLE, &car_config.length, 0, NULL},
		{(char *) "robot", (char *) "width", CARMEN_PARAM_DOUBLE, &car_config.width, 0, NULL},
		{(char *) "robot", (char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &car_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
		{(char *) "robot",  (char *) "distance_between_front_and_rear_axles",		CARMEN_PARAM_DOUBLE, &car_config.distance_between_front_and_rear_axles, 1, NULL},

		{(char *) "robot", (char *) "wheel_radius", CARMEN_PARAM_DOUBLE, &robot_wheel_radius, 0, NULL},

		{(char *) "sensor_board_1", (char *) "x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x),	0, NULL},
		{(char *) "sensor_board_1", (char *) "y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y),	0, NULL},
		{(char *) "sensor_board_1", (char *) "z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z),	0, NULL},
		{(char *) "sensor_board_1", (char *) "roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll),0, NULL},
		{(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch),0, NULL},
		{(char *) "sensor_board_1", (char *) "yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw),	0, NULL},

		{(char *) "velodyne", (char *) "x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
		{(char *) "velodyne", (char *) "y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
		{(char *) "velodyne", (char *) "z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
		{(char *) "velodyne", (char *) "roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
		{(char *) "velodyne", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
		{(char *) "velodyne", (char *) "yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

		{(char *) "localize_ackerman", (char *) "safe_range_above_sensors", CARMEN_PARAM_DOUBLE, &safe_range_above_sensors, 0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	carmen_param_allow_unfound_variables(1);

	carmen_param_t param_optional_list[] =
	{
		{(char *) "commandline", (char *) "calibration_file", CARMEN_PARAM_STRING, &calibration_file, 0, NULL}
	};

	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));

	sensor_board_1_to_car_matrix = create_rotation_matrix(sensor_board_1_pose.orientation);

	get_alive_sensors(argc, argv);

	get_sensors_param(argc, argv);

	localize_ackerman_velodyne_laser_read_parameters(argc, argv);
}


int 
register_ipc_messages(void)
{
	carmen_velodyne_define_messages();

	return 0;
}


static void
subscribe_to_ipc_message()
{
	if (spherical_sensor_params[0].alive)
		carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


int 
main(int argc, char **argv) 
{ 
	/* initialize carmen */
	carmen_randomize(&argc, &argv);
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	/* Setup exit handler */
	signal(SIGINT, shutdown_module);

	/* Initialize all the relevant parameters */
	read_parameters(argc, argv);

	register_ipc_messages();
	subscribe_to_ipc_message();

	carmen_ipc_dispatch();

	return 0;
}

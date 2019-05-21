/*
 * graphslam_params.cpp
 *
 *  Created on: Oct 10, 2013
 *      Author: filipe
 */
#include "graphslam_params.h"

double robot_length;
double robot_width;
double highest_sensor;
int use_raw_laser = 1;
double max_range = 0.0;
int correction_type = 0;
int number_of_sensors = 1;
double robot_wheel_radius;
carmen_pose_3D_t car_pose;
carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t gps_pose;
carmen_pose_3D_t xsens_pose;
double safe_range_above_sensors;
ProbabilisticMapParams map_params;
sensor_data_t *spherical_sensor_data;
carmen_pose_3D_t sensor_board_1_pose;
sensor_parameters_t *spherical_sensor_params;
rotation_matrix *sensor_board_1_to_car_matrix;
double distance_between_rear_car_and_rear_wheels;
double distance_between_front_and_rear_axles;
sensor_parameters_t velodyne_params;
sensor_data_t velodyne_data;
double highest_point;
bool use_fused_odometry = false;
int gps_to_use = 1;
double gps_latency = 0.0;

int number_of_threads = 1;


void
init_velodyne_points(spherical_point_cloud **velodyne_points_out, unsigned char ***intensity)
{
	int i;

	spherical_point_cloud *velodyne_points = (spherical_point_cloud *) calloc (NUM_VELODYNE_POINT_CLOUDS, sizeof(spherical_point_cloud));
	carmen_test_alloc(velodyne_points);

	*intensity = (unsigned char **)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(unsigned char*));
	carmen_test_alloc(*intensity);

	for (i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
	{
		velodyne_points[i].num_points = 0;
		velodyne_points[i].sphere_points = NULL;
	}

	*velodyne_points_out = velodyne_points;
}


void
get_alive_sensors(int argc, char **argv)
{
	int i;

	spherical_sensor_params = (sensor_parameters_t *)calloc(number_of_sensors, sizeof(sensor_parameters_t));
	carmen_test_alloc(spherical_sensor_params);

	spherical_sensor_data = (sensor_data_t *)calloc(number_of_sensors, sizeof(sensor_data_t));
	carmen_test_alloc(spherical_sensor_data);

	carmen_param_t param_list[] =
	{
			{(char*)"localize_ackerman", (char*)"velodyne", CARMEN_PARAM_ONOFF, &spherical_sensor_params[0].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne2", CARMEN_PARAM_ONOFF, &spherical_sensor_params[2].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne3", CARMEN_PARAM_ONOFF, &spherical_sensor_params[3].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne4", CARMEN_PARAM_ONOFF, &spherical_sensor_params[4].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne5", CARMEN_PARAM_ONOFF, &spherical_sensor_params[5].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne6", CARMEN_PARAM_ONOFF, &spherical_sensor_params[6].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne7", CARMEN_PARAM_ONOFF, &spherical_sensor_params[7].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne8", CARMEN_PARAM_ONOFF, &spherical_sensor_params[8].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne9", CARMEN_PARAM_ONOFF, &spherical_sensor_params[9].alive, 0, NULL},

			{(char*)"localize_ackerman", (char*)"velodyne_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne2_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne3_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne4_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne5_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne6_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne7_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne8_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne9_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].log_odds.log_odds_occ, 0, NULL},


			{(char*)"localize_ackerman", (char*)"velodyne_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne2_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne3_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne4_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne5_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne6_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne7_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne8_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne9_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].log_odds.log_odds_free, 0, NULL},

			{(char*)"localize_ackerman", (char*)"velodyne_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne2_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne3_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne4_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne5_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne6_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne7_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne8_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne9_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].log_odds.log_odds_l0, 0, NULL},

			{(char*)"localize_ackerman", (char*)"velodyne_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne2_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne3_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne4_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne5_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne6_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne7_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne8_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne9_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].unexpeted_delta_range_sigma, 0, NULL},

			{(char*)"localize_ackerman",  (char*)"velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].range_max_factor, 0, NULL},
			{(char*)"slam_icp", (char*) "highest_point", CARMEN_PARAM_DOUBLE, &highest_point, 0, NULL}

	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	for (i = 0; i < number_of_sensors; i++)
	{
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
}


int *
generates_ray_order(int size)
{
	int i;

	int *ray_order = (int *)malloc(size * sizeof(int));
	carmen_test_alloc(ray_order);

	for (i = 0; i < size; i++)
		ray_order[i] = i;

	return ray_order;
}


void
get_sensors_param(int argc, char **argv)
{
	int i, j;
	int flipped;
	int horizontal_resolution;
	char stereo_velodyne_string[256];

	int stereo_velodyne_vertical_roi_ini;
	int stereo_velodyne_vertical_roi_end;

	int stereo_velodyne_horizontal_roi_ini;
	int stereo_velodyne_horizontal_roi_end;

	int roi_ini, roi_end;

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
				{spherical_sensor_params[0].name, (char*)"vertical_resolution", CARMEN_PARAM_INT, &spherical_sensor_params[0].vertical_resolution, 0, NULL},
				{(char *)"localize_ackerman", (char*)"velodyne_range_max", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].range_max, 0, NULL},

		};

		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		init_velodyne_points(&spherical_sensor_data[0].points, &spherical_sensor_data[0].intensity);
		spherical_sensor_params[0].sensor_to_support_matrix = create_rotation_matrix(spherical_sensor_params[0].pose.orientation);
		spherical_sensor_data[0].point_cloud_index = 0;
		carmen_prob_models_alloc_sensor_data(&spherical_sensor_data[0], spherical_sensor_params[0].vertical_resolution, 1);

		if (max_range < spherical_sensor_params[0].range_max)
		{
			max_range = spherical_sensor_params[0].range_max;
		}
		spherical_sensor_params[0].current_range_max = spherical_sensor_params[0].range_max;
	}

	for (i = 1; i < number_of_sensors; i++)
	{
		if (spherical_sensor_params[i].alive)
		{
			spherical_sensor_params[i].pose = get_stereo_velodyne_pose_3D(argc, argv, i);

			spherical_sensor_params[i].sensor_robot_reference = carmen_change_sensor_reference(sensor_board_1_pose.position, spherical_sensor_params[i].pose.position, sensor_board_1_to_car_matrix);
			spherical_sensor_params[i].height = spherical_sensor_params[i].sensor_robot_reference.z + robot_wheel_radius;

			if (spherical_sensor_params[i].height > highest_sensor)
				highest_sensor = spherical_sensor_params[i].height;

			sprintf(stereo_velodyne_string, "%s%d", "stereo", i);


			carmen_param_t param_list[] =
			{
					{spherical_sensor_params[i].name, (char*) "vertical_resolution", CARMEN_PARAM_INT, &spherical_sensor_params[i].vertical_resolution, 0, NULL},
					{spherical_sensor_params[i].name, (char*) "horizontal_resolution", CARMEN_PARAM_INT, &horizontal_resolution, 0, NULL},
					{spherical_sensor_params[i].name, (char*) "flipped", CARMEN_PARAM_ONOFF, &flipped, 0, NULL},
					{spherical_sensor_params[i].name, (char*) "range_max", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[i].range_max, 0, NULL},
					{spherical_sensor_params[i].name, (char*) "vertical_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_ini, 0, NULL },
					{spherical_sensor_params[i].name, (char*) "vertical_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_end, 0, NULL },
					{spherical_sensor_params[i].name, (char*) "horizontal_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_ini, 0, NULL },
					{spherical_sensor_params[i].name, (char*) "horizontal_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_end, 0, NULL }

			};

			carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

			if (flipped)
			{
				spherical_sensor_params[i].vertical_resolution = horizontal_resolution;
				roi_ini = stereo_velodyne_horizontal_roi_ini;
				roi_end = stereo_velodyne_horizontal_roi_end;
			}
			else
			{
				roi_ini = stereo_velodyne_vertical_roi_ini;
				roi_end = stereo_velodyne_vertical_roi_end;
			}

			if (spherical_sensor_params[i].vertical_resolution > (roi_end - roi_ini))
			{
				carmen_die("The stereo_velodyne_vertical_resolution is bigger than stereo point cloud height");
			}

			if (max_range < spherical_sensor_params[i].range_max)
			{
				max_range = spherical_sensor_params[i].range_max;
			}

			spherical_sensor_params[i].current_range_max = spherical_sensor_params[i].range_max;

			spherical_sensor_params[i].range_max_factor = 1.0;
			spherical_sensor_params[i].ray_order = generates_ray_order(spherical_sensor_params[i].vertical_resolution);
			spherical_sensor_params[i].vertical_correction = get_stereo_velodyne_correction(flipped, i, spherical_sensor_params[i].vertical_resolution, roi_ini, roi_end, 0, 0);
			init_velodyne_points(&spherical_sensor_data[i].points, &spherical_sensor_data[i].intensity);
			spherical_sensor_params[i].sensor_to_support_matrix = create_rotation_matrix(spherical_sensor_params[i].pose.orientation);
			spherical_sensor_data[i].point_cloud_index = 0;
			carmen_prob_models_alloc_sensor_data(&spherical_sensor_data[i], spherical_sensor_params[i].vertical_resolution, 1);

			//TODO : tem que fazer esta medida para as cameras igual foi feito para o velodyne
			spherical_sensor_params[i].delta_difference_mean = (double *)calloc(50, sizeof(double));
			spherical_sensor_params[i].delta_difference_stddev = (double *)calloc(50, sizeof(double));
			for (j = 0; j < 50; j++)
				spherical_sensor_params[i].delta_difference_stddev[j] = 1.0;

		}
	}
}


void
read_parameters(int argc, char **argv, carmen_localize_ackerman_param_p param, ProbabilisticMapParams *p_map_params)
{
	double integrate_angle_deg;

	integrate_angle_deg = 1.0;

	carmen_param_t param_list[] =
	{
			{(char *)"robot", (char*)"frontlaser_offset", CARMEN_PARAM_DOUBLE, &param->front_laser_offset, 0, NULL},
			{(char *)"robot", (char*)"rearlaser_offset", CARMEN_PARAM_DOUBLE, &param->rear_laser_offset, 0, NULL},
			{(char *)"robot", (char*)"length", CARMEN_PARAM_DOUBLE, &robot_length, 0, NULL},
			{(char *)"robot", (char*)"width", CARMEN_PARAM_DOUBLE, &robot_width, 0, NULL},
			{(char *)"robot", (char*)"distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &distance_between_rear_car_and_rear_wheels, 1, NULL},
			{(char *)"robot", (char*)"distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &distance_between_front_and_rear_axles, 0, NULL},
			{(char *)"robot", (char*)"wheel_radius", CARMEN_PARAM_DOUBLE, &robot_wheel_radius, 0, NULL},

			{(char *)"localize", (char*)"use_rear_laser", CARMEN_PARAM_ONOFF, &param->use_rear_laser, 0, NULL},
			{(char *)"localize", (char*)"num_particles", CARMEN_PARAM_INT, &param->num_particles, 0, NULL},
			{(char *)"localize", (char*)"laser_max_range", CARMEN_PARAM_DOUBLE, &param->max_range, 1, NULL},
			{(char *)"localize", (char*)"min_wall_prob", CARMEN_PARAM_DOUBLE, &param->min_wall_prob, 0, NULL},
			{(char *)"localize", (char*)"outlier_fraction", CARMEN_PARAM_DOUBLE, &param->outlier_fraction, 0, NULL},
			{(char *)"localize", (char*)"update_distance", CARMEN_PARAM_DOUBLE, &param->update_distance, 0, NULL},
			{(char *)"localize", (char*)"integrate_angle_deg", CARMEN_PARAM_DOUBLE, &integrate_angle_deg, 0, NULL},
			{(char *)"localize", (char*)"do_scanmatching", CARMEN_PARAM_ONOFF, &param->do_scanmatching, 1, NULL},
			{(char *)"localize", (char*)"constrain_to_map", CARMEN_PARAM_ONOFF, &param->constrain_to_map, 1, NULL},
#ifdef OLD_MOTION_MODEL
			{(char *)"localize", (char*)"odom_a1", CARMEN_PARAM_DOUBLE, &param->odom_a1, 1, NULL},
			{(char *)"localize", (char*)"odom_a2", CARMEN_PARAM_DOUBLE, &param->odom_a2, 1, NULL},
			{(char *)"localize", (char*)"odom_a3", CARMEN_PARAM_DOUBLE, &param->odom_a3, 1, NULL},
			{(char *)"localize", (char*)"odom_a4", CARMEN_PARAM_DOUBLE, &param->odom_a4, 1, NULL},
#endif
			{(char *)"localize", (char*)"occupied_prob", CARMEN_PARAM_DOUBLE, &param->occupied_prob, 0, NULL},
			{(char *)"localize", (char*)"lmap_std", CARMEN_PARAM_DOUBLE, &param->lmap_std, 0, NULL},
			{(char *)"localize", (char*)"global_lmap_std", CARMEN_PARAM_DOUBLE, &param->global_lmap_std, 0, NULL},
			{(char *)"localize", (char*)"global_evidence_weight", CARMEN_PARAM_DOUBLE, &param->global_evidence_weight, 0, NULL},
			{(char *)"localize", (char*)"global_distance_threshold", CARMEN_PARAM_DOUBLE, &param->global_distance_threshold, 1, NULL},
			{(char *)"localize", (char*)"global_test_samples", CARMEN_PARAM_INT, &param->global_test_samples, 1, NULL},
			{(char *)"localize", (char*)"use_sensor", CARMEN_PARAM_ONOFF, &param->use_sensor, 0, NULL},
			{(char *)"localize", (char*)"tracking_beam_minlikelihood", CARMEN_PARAM_DOUBLE, &param->tracking_beam_minlikelihood, 0, NULL},
			{(char *)"localize", (char*)"global_beam_minlikelihood", CARMEN_PARAM_DOUBLE, &param->global_beam_minlikelihood, 0, NULL},

			{(char *)"sensor_board_1", (char*)"x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x),	0, NULL},
			{(char *)"sensor_board_1", (char*)"y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y),	0, NULL},
			{(char *)"sensor_board_1", (char*)"z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z),	0, NULL},
			{(char *)"sensor_board_1", (char*)"roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll),0, NULL},
			{(char *)"sensor_board_1", (char*)"pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch),0, NULL},
			{(char *)"sensor_board_1", (char*)"yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw),	0, NULL},

			{(char *) "car", 			  (char *) "x", 	CARMEN_PARAM_DOUBLE, &(car_pose.position.x), 0, NULL},
			{(char *) "car", 			  (char *) "y", 	CARMEN_PARAM_DOUBLE, &(car_pose.position.y), 0, NULL},
			{(char *) "car", 			  (char *) "z", 	CARMEN_PARAM_DOUBLE, &(car_pose.position.z), 0, NULL},
			{(char *) "car", 			  (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(car_pose.orientation.yaw), 0, NULL},
			{(char *) "car", 			  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(car_pose.orientation.pitch), 0, NULL},
			{(char *) "car", 			  (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(car_pose.orientation.roll), 0, NULL},

			{(char *)"velodyne", (char*)"x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
			{(char *)"velodyne", (char*)"y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
			{(char *)"velodyne", (char*)"z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
			{(char *)"velodyne", (char*)"roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
			{(char *)"velodyne", (char*)"pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
			{(char *)"velodyne", (char*)"yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

			{(char*)"localize_ackerman",  (char*)"number_of_sensors", CARMEN_PARAM_INT, &number_of_sensors, 0, NULL},
			{(char *)"localize_ackerman", (char*)"safe_range_above_sensors", CARMEN_PARAM_DOUBLE, &safe_range_above_sensors, 0, NULL},
			{(char *)"localize_ackerman", (char*)"correction_type", CARMEN_PARAM_INT, &correction_type, 0, NULL},

			{(char *)"mapper", (char*)"map_log_odds_max", CARMEN_PARAM_INT, &p_map_params->log_odds_max, 0, NULL},
			{(char *)"mapper", (char*)"map_log_odds_min", CARMEN_PARAM_INT, &p_map_params->log_odds_min, 0, NULL},
			{(char *)"mapper", (char*)"map_log_odds_bias", CARMEN_PARAM_INT, &p_map_params->log_odds_bias, 0, NULL},
			{(char *)"mapper", (char*)"map_grid_res", CARMEN_PARAM_DOUBLE, &p_map_params->grid_res, 0, NULL},
			{(char *)"mapper", (char*)"map_range_factor", CARMEN_PARAM_DOUBLE, &p_map_params->range_factor, 0, NULL},

			{(char*)"velodyne", (char*)"x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
			{(char*)"velodyne", (char*)"y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
			{(char*)"velodyne", (char*)"z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
			{(char*)"velodyne", (char*)"roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
			{(char*)"velodyne", (char*)"pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
			{(char*)"velodyne", (char*)"yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},
			{(char*)"velodyne", (char*)"vertical_resolution", CARMEN_PARAM_INT, &velodyne_params.vertical_resolution, 0, NULL},
			{(char*)"velodyne", (char*)"time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &velodyne_params.time_spent_by_each_scan, 0, NULL},
			{(char*)"localize_ackerman", (char*) "velodyne_laser_max_range", CARMEN_PARAM_DOUBLE, &velodyne_params.range_max, 0, NULL},
			{(char*)"slam_icp", (char*) "highest_point", CARMEN_PARAM_DOUBLE, &highest_point, 0, NULL},
	};


	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	param->integrate_angle = carmen_degrees_to_radians(integrate_angle_deg);

	sensor_board_1_to_car_matrix = create_rotation_matrix(sensor_board_1_pose.orientation);

	get_alive_sensors(argc, argv);

	get_sensors_param(argc, argv);

	p_map_params->width = 2 * max_range;
	p_map_params->height = 2 * max_range;
	p_map_params->grid_sx = p_map_params->width /  p_map_params->grid_res;
	p_map_params->grid_sy = p_map_params->height /  p_map_params->grid_res;
	p_map_params->grid_size = p_map_params->grid_sx * p_map_params->grid_sy;

	//localize_ackerman_velodyne_laser_read_parameters(argc, argv);

	carmen_param_allow_unfound_variables(1);

	carmen_param_t param_optional_list[] =
	{
		{(char *)"localize_ackerman", (char*)"use_raw_laser", CARMEN_PARAM_ONOFF, &use_raw_laser, 0, NULL},
	};

	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));
}


void
read_parameters_without_mapper(int argc, char **argv, carmen_localize_ackerman_param_p param, ProbabilisticMapParams *p_map_params, char *remission_file)
{
	double integrate_angle_deg;
	int i;
	carmen_pose_3D_t velodyne_pose;

	integrate_angle_deg = 1.0;

	carmen_param_t param_list[] =
	{
			{(char *)"robot", (char*)"frontlaser_offset", CARMEN_PARAM_DOUBLE, &param->front_laser_offset, 0, NULL},
			{(char *)"robot", (char*)"rearlaser_offset", CARMEN_PARAM_DOUBLE, &param->rear_laser_offset, 0, NULL},
			{(char *)"robot", (char*)"length", CARMEN_PARAM_DOUBLE, &robot_length, 0, NULL},
			{(char *)"robot", (char*)"width", CARMEN_PARAM_DOUBLE, &robot_width, 0, NULL},
			{(char *)"robot", (char*)"distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &distance_between_rear_car_and_rear_wheels, 1, NULL},

			{(char *)"robot", (char*)"wheel_radius", CARMEN_PARAM_DOUBLE, &robot_wheel_radius, 0, NULL},
			{(char *)"robot", (char*)"distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &distance_between_front_and_rear_axles, 0, NULL},
			{(char *)"localize", (char*)"use_rear_laser", CARMEN_PARAM_ONOFF, &param->use_rear_laser, 0, NULL},
			{(char *)"localize", (char*)"num_particles", CARMEN_PARAM_INT, &param->num_particles, 0, NULL},
			{(char *)"localize", (char*)"laser_max_range", CARMEN_PARAM_DOUBLE, &param->max_range, 1, NULL},
			{(char *)"localize", (char*)"min_wall_prob", CARMEN_PARAM_DOUBLE, &param->min_wall_prob, 0, NULL},
			{(char *)"localize", (char*)"outlier_fraction", CARMEN_PARAM_DOUBLE, &param->outlier_fraction, 0, NULL},
			{(char *)"localize", (char*)"update_distance", CARMEN_PARAM_DOUBLE, &param->update_distance, 0, NULL},
			{(char *)"localize", (char*)"integrate_angle_deg", CARMEN_PARAM_DOUBLE, &integrate_angle_deg, 0, NULL},
			{(char *)"localize", (char*)"do_scanmatching", CARMEN_PARAM_ONOFF, &param->do_scanmatching, 1, NULL},
			{(char *)"localize", (char*)"constrain_to_map", CARMEN_PARAM_ONOFF, &param->constrain_to_map, 1, NULL},
#ifdef OLD_MOTION_MODEL
			{(char *)"localize", (char*)"odom_a1", CARMEN_PARAM_DOUBLE, &param->odom_a1, 1, NULL},
			{(char *)"localize", (char*)"odom_a2", CARMEN_PARAM_DOUBLE, &param->odom_a2, 1, NULL},
			{(char *)"localize", (char*)"odom_a3", CARMEN_PARAM_DOUBLE, &param->odom_a3, 1, NULL},
			{(char *)"localize", (char*)"odom_a4", CARMEN_PARAM_DOUBLE, &param->odom_a4, 1, NULL},
#endif
			{(char *)"localize", (char*)"occupied_prob", CARMEN_PARAM_DOUBLE, &param->occupied_prob, 0, NULL},
			{(char *)"localize", (char*)"lmap_std", CARMEN_PARAM_DOUBLE, &param->lmap_std, 0, NULL},
			{(char *)"localize", (char*)"global_lmap_std", CARMEN_PARAM_DOUBLE, &param->global_lmap_std, 0, NULL},
			{(char *)"localize", (char*)"global_evidence_weight", CARMEN_PARAM_DOUBLE, &param->global_evidence_weight, 0, NULL},
			{(char *)"localize", (char*)"global_distance_threshold", CARMEN_PARAM_DOUBLE, &param->global_distance_threshold, 1, NULL},
			{(char *)"localize", (char*)"global_test_samples", CARMEN_PARAM_INT, &param->global_test_samples, 1, NULL},
			{(char *)"localize", (char*)"use_sensor", CARMEN_PARAM_ONOFF, &param->use_sensor, 0, NULL},
			{(char *)"localize", (char*)"tracking_beam_minlikelihood", CARMEN_PARAM_DOUBLE, &param->tracking_beam_minlikelihood, 0, NULL},
			{(char *)"localize", (char*)"global_beam_minlikelihood", CARMEN_PARAM_DOUBLE, &param->global_beam_minlikelihood, 0, NULL},

			{(char *)"sensor_board_1", (char*)"x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x),	0, NULL},
			{(char *)"sensor_board_1", (char*)"y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y),	0, NULL},
			{(char *)"sensor_board_1", (char*)"z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z),	0, NULL},
			{(char *)"sensor_board_1", (char*)"roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll),0, NULL},
			{(char *)"sensor_board_1", (char*)"pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch),0, NULL},
			{(char *)"sensor_board_1", (char*)"yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw),	0, NULL},

			{(char *) "car", 			  (char *) "x", 	CARMEN_PARAM_DOUBLE, &(car_pose.position.x), 0, NULL},
			{(char *) "car", 			  (char *) "y", 	CARMEN_PARAM_DOUBLE, &(car_pose.position.y), 0, NULL},
			{(char *) "car", 			  (char *) "z", 	CARMEN_PARAM_DOUBLE, &(car_pose.position.z), 0, NULL},
			{(char *) "car", 			  (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(car_pose.orientation.yaw), 0, NULL},
			{(char *) "car", 			  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(car_pose.orientation.pitch), 0, NULL},
			{(char *) "car", 			  (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(car_pose.orientation.roll), 0, NULL},

			{(char *)"velodyne", (char*)"x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
			{(char *)"velodyne", (char*)"y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
			{(char *)"velodyne", (char*)"z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
			{(char *)"velodyne", (char*)"roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
			{(char *)"velodyne", (char*)"pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
			{(char *)"velodyne", (char*)"yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},
			{(char *)"velodyne", (char*)"vertical_resolution", CARMEN_PARAM_INT, &velodyne_params.vertical_resolution, 0, NULL},
			{(char *)"velodyne", (char*)"time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &velodyne_params.time_spent_by_each_scan, 0, NULL},
			{(char *)"localize_ackerman", (char*) "velodyne_laser_max_range", CARMEN_PARAM_DOUBLE, &velodyne_params.range_max, 0, NULL},

			{(char *)"localize_ackerman",  (char*)"number_of_sensors", CARMEN_PARAM_INT, &number_of_sensors, 0, NULL},
			{(char *)"localize_ackerman", (char*)"safe_range_above_sensors", CARMEN_PARAM_DOUBLE, &safe_range_above_sensors, 0, NULL},
			{(char *)"localize_ackerman", (char*)"correction_type", CARMEN_PARAM_INT, &correction_type, 0, NULL},

			{(char *)"xsens", (char*)"x", CARMEN_PARAM_DOUBLE, &xsens_pose.position.x, 0, NULL},
			{(char *)"xsens", (char*)"y", CARMEN_PARAM_DOUBLE, &xsens_pose.position.y, 0, NULL},
			{(char *)"xsens", (char*)"z", CARMEN_PARAM_DOUBLE, &xsens_pose.position.z, 0, NULL},
			{(char *)"xsens", (char*)"roll", CARMEN_PARAM_DOUBLE, &xsens_pose.orientation.roll, 0, NULL},
			{(char *)"xsens", (char*)"pitch", CARMEN_PARAM_DOUBLE, &xsens_pose.orientation.pitch, 0, NULL},
			{(char *)"xsens", (char*)"yaw", CARMEN_PARAM_DOUBLE, &xsens_pose.orientation.yaw, 0, NULL},

			{(char *)"gps_nmea", (char*)"x", CARMEN_PARAM_DOUBLE, &gps_pose.position.x, 0, NULL},
			{(char *)"gps_nmea", (char*)"y", CARMEN_PARAM_DOUBLE, &gps_pose.position.y, 0, NULL},
			{(char *)"gps_nmea", (char*)"z", CARMEN_PARAM_DOUBLE, &gps_pose.position.z, 0, NULL},
			{(char *)"gps_nmea", (char*)"roll", CARMEN_PARAM_DOUBLE, &gps_pose.orientation.roll, 0, NULL},
			{(char *)"gps_nmea", (char*)"pitch", CARMEN_PARAM_DOUBLE, &gps_pose.orientation.pitch, 0, NULL},
			{(char *)"gps_nmea", (char*)"yaw", CARMEN_PARAM_DOUBLE, &gps_pose.orientation.yaw, 0, NULL},
			{(char *)"slam_icp", (char*) "highest_point", CARMEN_PARAM_DOUBLE, &highest_point, 0, NULL}
	};


	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
	param->integrate_angle = carmen_degrees_to_radians(integrate_angle_deg);
	sensor_board_1_to_car_matrix = create_rotation_matrix(sensor_board_1_pose.orientation);
	get_alive_sensors(argc, argv);
	get_sensors_param(argc, argv);
	//localize_ackerman_velodyne_laser_read_parameters(argc, argv);
	carmen_param_allow_unfound_variables(1);

	carmen_param_t param_optional_list[] =
	{
		{(char *)"localize_ackerman", (char *)"use_raw_laser", CARMEN_PARAM_ONOFF, &use_raw_laser, 0, NULL},
		{(char *)"commandline", 	  (char *)"use_fused_odometry", CARMEN_PARAM_ONOFF, &use_fused_odometry, 0, NULL},
		{(char *)"commandline", 	  (char *)"gps_to_use", CARMEN_PARAM_INT, &gps_to_use, 0, NULL},
		{(char *)"commandline", 	  (char *)"gps_latency", CARMEN_PARAM_DOUBLE, &gps_latency, 0, NULL},
	};

	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));

	param->integrate_angle = carmen_degrees_to_radians(integrate_angle_deg);
	sensor_board_1_to_car_matrix = create_rotation_matrix(sensor_board_1_pose.orientation);

	velodyne_params.alive = 1;

	velodyne_params.pose = velodyne_pose;
	velodyne_params.sensor_robot_reference = carmen_change_sensor_reference(sensor_board_1_pose.position, velodyne_params.pose.position, sensor_board_1_to_car_matrix);

	velodyne_params.height = velodyne_params.sensor_robot_reference.z + robot_wheel_radius;

	velodyne_params.ray_order = carmen_velodyne_get_ray_order();
	velodyne_params.vertical_correction = carmen_velodyne_get_vertical_correction();

	velodyne_params.delta_difference_mean = carmen_velodyne_get_delta_difference_mean();
	velodyne_params.delta_difference_stddev = carmen_velodyne_get_delta_difference_stddev();

	velodyne_params.sensor_to_support_matrix = create_rotation_matrix(velodyne_params.pose.orientation);
	velodyne_params.current_range_max = velodyne_params.range_max;

	velodyne_params.sensor_robot_reference = carmen_change_sensor_reference(sensor_board_1_pose.position, velodyne_params.pose.position, sensor_board_1_to_car_matrix);
	velodyne_params.height = velodyne_params.sensor_robot_reference.z + robot_wheel_radius;
	highest_sensor = 0.5;//velodyne_params.height;

	velodyne_params.remission_calibration = (double *) calloc(256 * velodyne_params.vertical_resolution, sizeof(double));

	FILE *f = fopen(remission_file, "r");
	for (i = 0; i < 256 * velodyne_params.vertical_resolution; i++)
	{
		fscanf(f, "%lf", &velodyne_params.remission_calibration[i]);
	}
	fclose(f);

	velodyne_data.point_cloud_index = 0;

	velodyne_data.ray_position_in_the_floor = (carmen_vector_2D_t**)calloc(number_of_threads ,sizeof(carmen_vector_2D_t*));
	velodyne_data.maxed = (int**)calloc(number_of_threads ,sizeof(int*));
	velodyne_data.obstacle_height = (double**)calloc(number_of_threads ,sizeof(double*));
	velodyne_data.occupancy_log_odds_of_each_ray_target = (double**)calloc(number_of_threads ,sizeof(double*));
	velodyne_data.point_cloud_index = 0;
	velodyne_data.points = NULL;
	velodyne_data.ray_origin_in_the_floor = (carmen_vector_2D_t**)calloc(number_of_threads ,sizeof(carmen_vector_2D_t*));;
	velodyne_data.ray_size_in_the_floor = (double**)calloc(number_of_threads ,sizeof(double*));
	velodyne_data.processed_intensity = (double**)calloc(number_of_threads ,sizeof(double*));
	velodyne_data.ray_hit_the_robot = (int**)calloc(number_of_threads ,sizeof(int*));
	velodyne_data.ray_that_hit_the_nearest_target = (int*)calloc(number_of_threads ,sizeof(int));

	for (int j = 0; j < number_of_threads; j++)
	{
		velodyne_data.ray_position_in_the_floor[j] = NULL;
		velodyne_data.maxed[j] = NULL;
		velodyne_data.obstacle_height[j] = NULL;
		velodyne_data.occupancy_log_odds_of_each_ray_target[j] = NULL;
		velodyne_data.ray_origin_in_the_floor[j] = NULL;
		velodyne_data.ray_size_in_the_floor[j] = NULL;
		velodyne_data.processed_intensity[i] = NULL;
		velodyne_data.ray_hit_the_robot[j] = NULL;
	}

	p_map_params->width = 2 * velodyne_params.range_max;
	p_map_params->height = 2 * velodyne_params.range_max;
	p_map_params->grid_sx = p_map_params->width /  p_map_params->grid_res;
	p_map_params->grid_sy = p_map_params->height /  p_map_params->grid_res;
	p_map_params->grid_size = p_map_params->grid_sx * p_map_params->grid_sy;

	carmen_prob_models_alloc_sensor_data(&velodyne_data, velodyne_params.vertical_resolution, 1);
	carmen_param_allow_unfound_variables(1);
}

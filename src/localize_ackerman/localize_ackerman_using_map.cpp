#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <carmen/grid_mapping.h>
#include "localize_ackerman_velodyne.h"
#include "localize_ackerman_using_map.h"

#define	UPDATE_CELLS_CROSSED_BY_RAYS		1
#define	DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS	0

extern double safe_range_above_sensors;
extern double robot_wheel_radius;

extern double highest_sensor;

extern carmen_pose_3D_t sensor_board_1_pose;
extern rotation_matrix *sensor_board_1_to_car_matrix;

extern sensor_parameters_t *spherical_sensor_params;
extern sensor_data_t *spherical_sensor_data;

carmen_pose_3D_t robot_pose;
carmen_vector_3D_t robot_velocity;
double robot_phi;
double robot_pose_timestamp;

carmen_map_t sum_remission_map, sum_sqr_remission_map, count_remission_map, current_mean_remission_map, current_variance_remission_map;

rotation_matrix *r_matrix_car_to_global = NULL;

int globalpos_initialized = 0;

extern carmen_robot_ackerman_config_t car_config;

extern carmen_compact_map_t local_compacted_mean_remission_map;
extern carmen_compact_map_t local_compacted_variance_remission_map;

carmen_map_config_t map_config;

carmen_position_t g_map_origin;


static void
change_sensor_rear_range_max(sensor_parameters_t *sensor_params, double angle)
{
	if ((angle > M_PI / 2.0) || (angle < -M_PI / 2.0))
		sensor_params->current_range_max = sensor_params->range_max / sensor_params->range_max_factor;
	else
		sensor_params->current_range_max = sensor_params->range_max;
}


static void
update_cells_in_the_velodyne_perceptual_field(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global,
					      int point_cloud_index)
{
	int i, j;
	spherical_point_cloud v_zt = sensor_data->points[point_cloud_index];
	carmen_pose_3D_t robot_interpolated_position;

	double v = sensor_data->robot_velocity[point_cloud_index].x;
	double phi = sensor_data->robot_phi[point_cloud_index];

	double scan_time = (v_zt.num_points / sensor_params->vertical_resolution) * sensor_params->time_spent_by_each_scan;
	double dt = (sensor_data->current_timestamp - sensor_data->robot_timestamp[point_cloud_index]) - scan_time;

	// Ray-trace the grid
	for (i = 0, j = 0; i < v_zt.num_points; i = i +  sensor_params->vertical_resolution, j++, dt += sensor_params->time_spent_by_each_scan)
	{
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(sensor_data->robot_pose[point_cloud_index], dt, v, phi, car_config.distance_between_front_and_rear_axles);
		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

		change_sensor_rear_range_max(sensor_params, v_zt.sphere_points[i].horizontal_angle);

		carmen_prob_models_compute_relevant_map_coordinates(sensor_data, sensor_params, i, robot_interpolated_position.position, sensor_board_1_pose,
				r_matrix_robot_to_global, sensor_board_1_to_car_matrix, robot_wheel_radius, g_map_origin.x, g_map_origin.y, &car_config, 0, 0);

		carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(sensor_data, sensor_params, i,
				highest_sensor, safe_range_above_sensors, 0, 0);

		carmen_prob_models_update_intensity_of_cells_hit_by_rays(&sum_remission_map, &sum_sqr_remission_map, &count_remission_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors, NULL, 0);
	}
}


static void
build_map_using_velodyne(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global)
{
	update_cells_in_the_velodyne_perceptual_field(sensor_params, sensor_data, r_matrix_robot_to_global, sensor_data->point_cloud_index);
	carmen_prob_models_calc_mean_and_variance_remission_map(&current_mean_remission_map, &current_variance_remission_map, &sum_remission_map, &sum_sqr_remission_map, &count_remission_map);

	carmen_prob_models_free_compact_map(&local_compacted_mean_remission_map);
	carmen_prob_models_free_compact_map(&local_compacted_variance_remission_map);

	carmen_prob_models_create_compact_map(&local_compacted_mean_remission_map, &current_mean_remission_map, -1.0);
	carmen_prob_models_create_compact_map(&local_compacted_variance_remission_map, &current_variance_remission_map, -1.0);

	double sin_theta = sin(robot_pose.orientation.yaw);
	double cos_theta = cos(robot_pose.orientation.yaw);


	for (int i = 0; i < local_compacted_mean_remission_map.number_of_known_points_on_the_map; i++)
	{
		double x = (robot_pose.position.x - g_map_origin.x) / 0.2;
		double y = (robot_pose.position.y - g_map_origin.y) / 0.2;

		double dx = (double)local_compacted_mean_remission_map.coord_x[i] - x;// - (double)local_compacted_mean_remission_map.config.x_size / 2.0;
		double dy = (double)local_compacted_mean_remission_map.coord_y[i] - y;// - (double)local_compacted_mean_remission_map.config.y_size / 2.0;

		double dxg = dx * cos_theta + dy * sin_theta;
		double dyg = -dx * sin_theta + dy * cos_theta;

		local_compacted_mean_remission_map.coord_x[i] = (int)((dxg) + (double)local_compacted_mean_remission_map.config.x_size / 2.0);
		local_compacted_mean_remission_map.coord_y[i] = (int)((dyg) + (double)local_compacted_mean_remission_map.config.y_size / 2.0);
	}
}


void
localize_using_map_change_map_origin_to_another_map_block(carmen_position_t *map_origin)
{
	static int first_time = 1;

	static carmen_map_t new_sum_remission_map, new_sum_sqr_remission_map, new_count_remission_map;

	if (first_time)
	{
		g_map_origin = (*map_origin);

		carmen_grid_mapping_create_new_map(&new_sum_remission_map, sum_remission_map.config.x_size, sum_remission_map.config.y_size, sum_remission_map.config.resolution, 's');
		carmen_grid_mapping_create_new_map(&new_sum_sqr_remission_map, sum_sqr_remission_map.config.x_size, sum_sqr_remission_map.config.y_size, sum_sqr_remission_map.config.resolution, '2');
		carmen_grid_mapping_create_new_map(&new_count_remission_map, count_remission_map.config.x_size, count_remission_map.config.y_size, count_remission_map.config.resolution, 'c');
		first_time = 0;
	}

	//verify if its necessery to change the map
	if (carmen_grid_mapping_is_map_changed(map_origin, g_map_origin.x, g_map_origin.y))
	{
		g_map_origin = *map_origin;

		carmen_point_t pose;
		pose.x = robot_pose.position.x;
		pose.y = robot_pose.position.y;
		pose.theta = robot_pose.orientation.yaw;

		carmen_grid_mapping_change_blocks_map_by_origin(pose, &sum_remission_map, &new_sum_remission_map);
		carmen_grid_mapping_change_blocks_map_by_origin(pose, &sum_sqr_remission_map, &new_sum_sqr_remission_map);
		carmen_grid_mapping_change_blocks_map_by_origin(pose, &count_remission_map, &new_count_remission_map);

		carmen_grid_mapping_swap_maps_and_clear_old_map(&sum_remission_map, &new_sum_remission_map);
		carmen_grid_mapping_swap_maps_and_clear_old_map(&sum_sqr_remission_map, &new_sum_sqr_remission_map);
		carmen_grid_mapping_swap_maps_and_clear_old_map(&count_remission_map, &new_count_remission_map);

		sum_remission_map.config.x_origin = g_map_origin.x;
		sum_remission_map.config.y_origin = g_map_origin.y;

		sum_sqr_remission_map.config.x_origin = g_map_origin.x;
		sum_sqr_remission_map.config.y_origin = g_map_origin.y;

		count_remission_map.config.x_origin = g_map_origin.x;
		count_remission_map.config.y_origin = g_map_origin.y;
	}
}


//void
//localize_using_map_change_map_origin(double map_displacement_x, double map_displacement_y)
//{
//	static int first_time = 1;
//
//	static carmen_map_t new_sum_remission_map, new_sum_sqr_remission_map, new_count_remission_map;
//
//	if (first_time)
//	{
//		carmen_grid_mapping_create_new_map(&new_sum_remission_map, sum_remission_map.config.x_size, sum_remission_map.config.y_size, sum_remission_map.config.resolution);
//		carmen_grid_mapping_create_new_map(&new_sum_sqr_remission_map, sum_sqr_remission_map.config.x_size, sum_sqr_remission_map.config.y_size, sum_sqr_remission_map.config.resolution);
//		carmen_grid_mapping_create_new_map(&new_count_remission_map, count_remission_map.config.x_size, count_remission_map.config.y_size, count_remission_map.config.resolution);
//		first_time = 0;
//	}
//
//	g_map_origin.x += map_displacement_x;
//	g_map_origin.y += map_displacement_y;
//
//	for (int x = 0; x < sum_remission_map.config.x_size; x++)
//	{
//		int x_new = x + (int)((map_displacement_x / sum_remission_map.config.resolution) + 0.5);
//		if (x_new > 0 && x_new < sum_remission_map.config.x_size)
//			for (int y = 0; y < sum_remission_map.config.y_size; y++)
//			{
//				int y_new = y + (int)((map_displacement_y / sum_remission_map.config.resolution) + 0.5);
//				if (y_new > 0 && y_new < sum_remission_map.config.y_size)
//				{
//					new_sum_remission_map.map[x][y] = sum_remission_map.map[x_new][y_new];
//					new_sum_sqr_remission_map.map[x][y] = sum_sqr_remission_map.map[x_new][y_new];
//					new_count_remission_map.map[x][y] = count_remission_map.map[x_new][y_new];
//				}
//			}
//	}
//
//	carmen_grid_mapping_swap_maps_and_clear_old_map(&sum_remission_map, &new_sum_remission_map);
//	carmen_grid_mapping_swap_maps_and_clear_old_map(&sum_sqr_remission_map, &new_sum_sqr_remission_map);
//	carmen_grid_mapping_swap_maps_and_clear_old_map(&count_remission_map, &new_count_remission_map);
//
//	sum_remission_map.config.x_origin = g_map_origin.x;
//	sum_remission_map.config.y_origin = g_map_origin.y;
//
//	sum_sqr_remission_map.config.x_origin = g_map_origin.x;
//	sum_sqr_remission_map.config.y_origin = g_map_origin.y;
//
//	count_remission_map.config.x_origin = g_map_origin.x;
//	count_remission_map.config.y_origin = g_map_origin.y;
//}


static int
run_localize_using_map(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global)
{
	static carmen_point_t world_pose;
	static int first = 1;
	carmen_position_t map_origin;

	if (!globalpos_initialized)
		return 0;

	if (first)
	{
		world_pose.x = robot_pose.position.x;
		world_pose.y = robot_pose.position.y;
		world_pose.theta = robot_pose.orientation.yaw;
		first = 0;
		return 0;
	}

//	localize_using_map_change_map_origin((robot_pose.position.x - world_pose.x), (robot_pose.position.y - world_pose.y));

	world_pose.x = robot_pose.position.x;
	world_pose.y = robot_pose.position.y;
	world_pose.theta = robot_pose.orientation.yaw;

	carmen_grid_mapping_get_map_origin(&world_pose, &map_origin.x, &map_origin.y);

	localize_using_map_change_map_origin_to_another_map_block(&map_origin);

	build_map_using_velodyne(sensor_params, sensor_data, r_matrix_robot_to_global);
	return 1;
}


static void
build_sensor_point_cloud(spherical_point_cloud **points, unsigned char **intensity, int *point_cloud_index, int num_points, int max_point_buffer)
{
	(*point_cloud_index)++;
	if ((*point_cloud_index) >= max_point_buffer)
		*point_cloud_index = 0;

	if ((*points)[*point_cloud_index].num_points != num_points)
		intensity[*point_cloud_index] = (unsigned char *)realloc((void *)intensity[*point_cloud_index], num_points * sizeof(unsigned char));

	carmen_alloc_spherical_point_cloud(*points, num_points, *point_cloud_index);
}


int
localize_using_map_velodyne_variable_scan(int sensor_number, carmen_velodyne_variable_scan_message *message)
{
	static int message_id;
	int ok_to_publish;

	int num_points = message->number_of_shots * spherical_sensor_params[sensor_number].vertical_resolution;

	ok_to_publish = 0;
	if (!globalpos_initialized)
		return (ok_to_publish);

	if (spherical_sensor_data[sensor_number].last_timestamp == 0.0)
	{
		spherical_sensor_data[sensor_number].last_timestamp = message->timestamp;
		message_id = -2;		// correntemente sďż˝o necessďż˝rias pelo menos 2 mensagens para ter uma volta completa de velodyne

		return (ok_to_publish);
	}
	
	spherical_sensor_data[sensor_number].last_timestamp = spherical_sensor_data[sensor_number].current_timestamp = message->timestamp;

	build_sensor_point_cloud(&spherical_sensor_data[sensor_number].points, spherical_sensor_data[sensor_number].intensity, &spherical_sensor_data[sensor_number].point_cloud_index, num_points, NUM_VELODYNE_POINT_CLOUDS);

	carmen_velodyne_variable_scan_update_points(message, spherical_sensor_params[sensor_number].vertical_resolution,
			&spherical_sensor_data[sensor_number].points[spherical_sensor_data[sensor_number].point_cloud_index],
			spherical_sensor_data[sensor_number].intensity[spherical_sensor_data[sensor_number].point_cloud_index],
			spherical_sensor_params[sensor_number].ray_order, spherical_sensor_params[sensor_number].vertical_correction,
			spherical_sensor_params[sensor_number].range_max,
			message->timestamp);

	spherical_sensor_data[sensor_number].robot_pose[spherical_sensor_data[sensor_number].point_cloud_index] = robot_pose;
	spherical_sensor_data[sensor_number].robot_velocity[spherical_sensor_data[sensor_number].point_cloud_index] = robot_velocity;
	spherical_sensor_data[sensor_number].robot_timestamp[spherical_sensor_data[sensor_number].point_cloud_index] = robot_pose_timestamp;
	spherical_sensor_data[sensor_number].robot_phi[spherical_sensor_data[sensor_number].point_cloud_index] = robot_phi;

	if (message_id >= 0)
	{
		ok_to_publish = run_localize_using_map(&spherical_sensor_params[sensor_number], &spherical_sensor_data[sensor_number], r_matrix_car_to_global);

		if (message_id > 1000000)
			message_id = 0;
	}
	message_id++;
	spherical_sensor_data[sensor_number].last_timestamp = message->timestamp;
	
	return (ok_to_publish);
}


void
localize_using_map_set_robot_pose_into_the_map(double v, double phi, double timestamp)
{
	static double last_timestamp = timestamp;

	globalpos_initialized = 1;

	robot_pose = carmen_ackerman_interpolated_robot_position_at_time(robot_pose, timestamp - last_timestamp, v, phi, car_config.distance_between_front_and_rear_axles);

	r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_pose.orientation);

	last_timestamp = timestamp;
}


void
localize_using_map_initialize(carmen_map_config_t *main_map_config)
{
	map_config = *main_map_config;
	
	carmen_grid_mapping_create_new_map(&sum_remission_map, map_config.x_size, map_config.y_size, map_config.resolution, 's');
	carmen_grid_mapping_create_new_map(&sum_sqr_remission_map, map_config.x_size, map_config.y_size, map_config.resolution, '2');
	carmen_grid_mapping_create_new_map(&count_remission_map, map_config.x_size, map_config.y_size, map_config.resolution, 'c');
	carmen_grid_mapping_create_new_map(&current_mean_remission_map, map_config.x_size, map_config.y_size, map_config.resolution, 'm');
	carmen_grid_mapping_create_new_map(&current_variance_remission_map, map_config.x_size, map_config.y_size, map_config.resolution, 'm');

	carmen_grid_mapping_init_parameters(map_config.resolution, map_config.x_size * map_config.resolution);

	memset(&robot_pose, 0, sizeof(carmen_pose_3D_t));
	carmen_point_t pose;

	pose.x = robot_pose.position.x = (main_map_config->x_size * main_map_config->resolution) / 2.0;
	pose.y = robot_pose.position.y = (main_map_config->y_size * main_map_config->resolution) / 2.0;


	carmen_grid_mapping_get_map_origin(&pose, &g_map_origin.x, &g_map_origin.y);
}

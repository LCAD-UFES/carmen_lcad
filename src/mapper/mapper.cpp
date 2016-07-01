#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <carmen/grid_mapping.h>
#include <carmen/grid_mapping_interface.h>

#include "mapper.h"

#define	UPDATE_CELLS_CROSSED_BY_RAYS		1
#define	DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS	0

extern double safe_range_above_sensors;
extern double robot_wheel_radius;

extern double highest_sensor;

extern int merge_with_offline_map;
extern int update_and_merge_with_mapper_saved_maps;
extern int build_snapshot_map;
extern int update_and_merge_with_snapshot_map;

extern carmen_pose_3D_t sensor_board_1_pose;
extern rotation_matrix *board_to_car_matrix;

extern sensor_parameters_t *sensors_params;
extern sensor_data_t *sensors_data;
extern int number_of_sensors;

extern char *map_path;

extern int publish_moving_objects_raw_map;

extern int robot_near_bump_or_barrier;
extern double obstacle_cost_distance, obstacle_probability_threshold;
extern int ok_to_publish;


/**
 * The map
 */

carmen_map_t map, sum_remission_map, sum_sqr_remission_map, count_remission_map, moving_objects_raw_map;

carmen_map_t cost_map;

extern carmen_map_t offline_map;

rotation_matrix *r_matrix_car_to_global = NULL;

int globalpos_initialized = 0;
extern carmen_localize_ackerman_globalpos_message *globalpos_history;
extern int last_globalpos;

carmen_robot_ackerman_config_t car_config;
carmen_map_config_t map_config;

double x_origin, y_origin; // map origin in meters

static carmen_laser_laser_message flaser; // moving_objects


static void
change_sensor_rear_range_max(sensor_parameters_t *sensor_params, double angle)
{
	if ((angle > M_PI / 2.0) || (angle < -M_PI / 2.0))
		sensor_params->current_range_max = sensor_params->range_max / sensor_params->range_max_factor;
	else
		sensor_params->current_range_max = sensor_params->range_max;
}

// Inicio do teste: moving_objects - Eduardo
void
publish_frontlaser(double timestamp)
{
	IPC_RETURN_TYPE err = IPC_OK;

	//carmen_simulator_ackerman_calc_laser_msg(&flaser, simulator_config, 0);

	flaser.timestamp = timestamp;
	err = IPC_publishData(CARMEN_LASER_FRONTLASER_NAME, &flaser);
	carmen_test_ipc(err, "Could not publish laser_frontlaser_message",
			CARMEN_LASER_FRONTLASER_NAME);
}

void
build_front_laser_message_from_velodyne_point_cloud(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, spherical_point_cloud v_zt, int i)
{
	static int first = 1;

	if (first)
	{
		flaser.host = carmen_get_host();
		flaser.num_readings = 720;
		flaser.range = (double *)calloc
				(720, sizeof(double));
		carmen_test_alloc(flaser.range);

		flaser.num_remissions = 0;
		flaser.remission = 0;

		flaser.config.angular_resolution = 0.5;
		flaser.config.fov = sensor_params->fov_range;
		flaser.config.maximum_range = sensor_params->range_max;
		flaser.config.remission_mode = REMISSION_NONE;
		flaser.config.start_angle = sensor_params->start_angle;
		first = 0;
	}

	int laser_ray_angle_index = 0;
	laser_ray_angle_index = (int)(v_zt.sphere_points[i].horizontal_angle / 0.5) % (720);

	//if (carmen_prob_models_log_odds_to_probabilistic(sensor_data->occupancy_log_odds_of_each_ray_target[sensor_data->ray_that_hit_the_nearest_target]) > 0.95)
	flaser.range[laser_ray_angle_index] = sensor_data->ray_size_in_the_floor[sensor_data->ray_that_hit_the_nearest_target];

	if (sensor_data->maxed[sensor_data->ray_that_hit_the_nearest_target])
			flaser.range[laser_ray_angle_index] = sensor_params->current_range_max;

//	printf("%lf ", flaser.range[laser_ray_angle_index]);

}

//teste: fim

int
get_nearest_timestamp_index(double *robot_timestamp, spherical_point_cloud *points, int cindex)
{
	int index_nearest_timestamp = 0;
	double timestamp_diff = 10.0;
	int j = cindex;

	for (int i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
	{
		double diff = fabs(robot_timestamp[j] - points[i].timestamp);
		//printf("diff = %lf, rt = %lf, vt = %lf\n", robot_timestamp[j] - points[i].timestamp, robot_timestamp[j], points[i].timestamp);
		if (diff < timestamp_diff)
		{
			timestamp_diff = diff;
			index_nearest_timestamp = i;
		}
	}
//
//	if (timestamp_diff != 0.0)
//	{
//		j = ((cindex - 1) < 0)? NUM_VELODYNE_POINT_CLOUDS - 1: cindex - 1;
//		for (int i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
//		{
//			double diff = fabs(robot_timestamp[j] - points[i].timestamp);
//			//printf("diff = %lf, rt = %lf, vt = %lf\n", robot_timestamp[j] - points[i].timestamp, robot_timestamp[j], points[i].timestamp);
//			if (diff < timestamp_diff)
//			{
//				timestamp_diff = diff;
//				index_nearest_timestamp = i;
//			}
//		}
//	}

	//printf("time diff = %lf, index = %d, cindex = %d\n", timestamp_diff, index_nearest_timestamp, cindex);

	return (index_nearest_timestamp);
}


static void
update_cells_in_the_velodyne_perceptual_field(carmen_map_t *snapshot_map, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global,
					      int point_cloud_index, int update_cells_crossed_by_rays, int build_snapshot_map)
{
	int nearest_timestamp_index = get_nearest_timestamp_index(sensor_data->robot_timestamp,
			sensor_data->points, point_cloud_index);
	spherical_point_cloud v_zt = sensor_data->points[nearest_timestamp_index];

	double v = sensor_data->robot_velocity[point_cloud_index].x;
	double phi = sensor_data->robot_phi[point_cloud_index];
	//int N = v_zt.num_points / sensor_params->vertical_resolution;

	//double dt = 0.0494 / (double) N; // @@@ Alberto: este dt depende da velocidade de rotação do Velodyne, que não é fixa. Tem que ser calculado do acordo com a velocidade de rotação do Velodyne.
	double dt = 1.0 / (1808.0 * 12.0);
	carmen_pose_3D_t robot_interpolated_position = sensor_data->robot_pose[point_cloud_index];

	// Ray-trace the grid
	for (int i = 0, j = 0; i < v_zt.num_points; i = i +  sensor_params->vertical_resolution, j++)
	{
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(robot_interpolated_position, dt, v, phi, car_config.distance_between_front_and_rear_axles);
		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

		change_sensor_rear_range_max(sensor_params, v_zt.sphere_points[i].horizontal_angle);

		carmen_prob_models_compute_relevant_map_coordinates(sensor_data, sensor_params, i, robot_interpolated_position.position, sensor_board_1_pose,
				r_matrix_robot_to_global, board_to_car_matrix, robot_wheel_radius, x_origin, y_origin, &car_config, robot_near_bump_or_barrier);

		carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(sensor_data, sensor_params, i, highest_sensor, safe_range_above_sensors,
				robot_near_bump_or_barrier);

		if (update_cells_crossed_by_rays == UPDATE_CELLS_CROSSED_BY_RAYS)
			carmen_prob_models_update_cells_crossed_by_ray(snapshot_map, sensor_params, sensor_data);

			// carmen_prob_models_upgrade_log_odds_of_cells_hit_by_rays(map, sensor_params, sensor_data);
		carmen_prob_models_update_log_odds_of_cells_hit_by_rays(snapshot_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors);
		carmen_prob_models_update_intensity_of_cells_hit_by_rays(&sum_remission_map, &sum_sqr_remission_map, &count_remission_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors, NULL);

		//Lucas: Mapa para deteccao de objetos moveis
		carmen_prob_models_update_log_odds_of_cells_hit_by_rays(&moving_objects_raw_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors);
		//build_front_laser_message_from_velodyne_point_cloud (sensor_params, sensor_data, v_zt, i);
	}
}


void
set_map_equal_offline_map(carmen_map_t *current_map)
{
	int xi, yi;

	for (xi = 0; xi < current_map->config.x_size; xi++)
		for (yi = 0; yi < current_map->config.y_size; yi++)
			current_map->map[xi][yi] = offline_map.map[xi][yi];
}


void
add_offline_map_over_unknown(carmen_map_t *current_map)
{
	int xi, yi;

	for (xi = 0; xi < current_map->config.x_size; xi++)
		for (yi = 0; yi < current_map->config.y_size; yi++)
			if (current_map->map[xi][yi] < 0.0)
				current_map->map[xi][yi] = offline_map.map[xi][yi];
}


void
map_decay_to_offline_map(carmen_map_t *current_map)
{
	int xi, yi;

	for (xi = 0; xi < current_map->config.x_size; xi++)
	{
		for (yi = 0; yi < current_map->config.y_size; yi++)
		{
			if (current_map->map[xi][yi] >= 0.0)
			{
				//current_map->map[xi][yi] = (50.0 * current_map->map[xi][yi] + offline_map.map[xi][yi]) / 51.0;
				current_map->map[xi][yi] = (10.0 * current_map->map[xi][yi] + offline_map.map[xi][yi]) / 11.0;
				//current_map->map[xi][yi] = carmen_prob_models_log_odds_to_probabilistic((get_log_odds(current_map->map[xi][yi]) + get_log_odds(offline_map.map[xi][yi])) / 2.0);
				//if (fabs(current_map->map[xi][yi] - 0.5) < 0.1)
				//	current_map->map[xi][yi] = -1.0;
			}
			else
				current_map->map[xi][yi] = offline_map.map[xi][yi];
		}
	}
}


static void
build_map_using_velodyne(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global)
{
	static carmen_map_t *snapshot_map = NULL;

	snapshot_map = carmen_prob_models_check_if_new_snapshot_map_allocation_is_needed(snapshot_map, &map);
	//set_map_equal_offline_map(&map);
//	add_offline_map_over_unknown(&map);


	map_decay_to_offline_map(&map);

	// @@@ Alberto: Mapa padrao Lucas -> colocar DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS ao inves de UPDATE_CELLS_CROSSED_BY_RAYS
	//update_cells_in_the_velodyne_perceptual_field(&map, snapshot_map, sensor_params, sensor_data, r_matrix_robot_to_global, sensor_data->point_cloud_index, DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS, update_and_merge_with_snapshot_map);
	update_cells_in_the_velodyne_perceptual_field(snapshot_map, sensor_params, sensor_data, r_matrix_robot_to_global, sensor_data->point_cloud_index, UPDATE_CELLS_CROSSED_BY_RAYS, update_and_merge_with_snapshot_map);
	carmen_prob_models_update_current_map_with_snapshot_map_and_clear_snapshot_map(&map, snapshot_map);
	//if (build_snapshot_map)


//	add_offline_map_over_unknown(&map);
}


void
initialize_first_map_block_origin(carmen_map_t *current_carmen_map, carmen_position_t *map_origin, char map_type)
{
	current_carmen_map->complete_map = NULL;
	x_origin = map_origin->x;
	y_origin = map_origin->y;

	if (update_and_merge_with_mapper_saved_maps)
	{
		carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, map_type, x_origin, y_origin, current_carmen_map);

		if (current_carmen_map->complete_map == NULL)
			carmen_grid_mapping_initialize_map(current_carmen_map, map.config.x_size, map.config.resolution);
	}
	else		
		carmen_grid_mapping_get_buffered_map(x_origin, y_origin, current_carmen_map);
}



void
mapper_change_map_origin_to_another_map_block(carmen_position_t *map_origin)
{
	static int first_time = 1;

	static carmen_map_t new_carmen_map, new_sum_remission_map, new_sum_sqr_remission_map, new_count_remission_map;

	if (first_time)
	{
		initialize_first_map_block_origin(&map, map_origin, 'm');
		initialize_first_map_block_origin(&moving_objects_raw_map, map_origin, 'm');
		initialize_first_map_block_origin(&sum_remission_map, map_origin, 's');
		initialize_first_map_block_origin(&sum_sqr_remission_map, map_origin, '2');
		initialize_first_map_block_origin(&count_remission_map, map_origin, 'c');

		carmen_grid_mapping_create_new_map(&new_carmen_map, map.config.x_size, map.config.y_size, map.config.resolution);
		carmen_grid_mapping_create_new_map(&new_sum_remission_map, sum_remission_map.config.x_size, sum_remission_map.config.y_size, sum_remission_map.config.resolution);
		carmen_grid_mapping_create_new_map(&new_sum_sqr_remission_map, sum_sqr_remission_map.config.x_size, sum_sqr_remission_map.config.y_size, sum_sqr_remission_map.config.resolution);
		carmen_grid_mapping_create_new_map(&new_count_remission_map, count_remission_map.config.x_size, count_remission_map.config.y_size, count_remission_map.config.resolution);

		first_time = 0;
	}

	//verify if its necessery to change the map
	if (carmen_grid_mapping_is_map_changed(map_origin, x_origin, y_origin))
	{
		x_origin = map_origin->x;
		y_origin = map_origin->y;

		if (update_and_merge_with_mapper_saved_maps)
		{
			//save the current map in the given map_path
			carmen_grid_mapping_save_block_map_by_origin(map_path, 'm', &map);
			carmen_grid_mapping_save_block_map_by_origin(map_path, 's', &sum_remission_map);
			carmen_grid_mapping_save_block_map_by_origin(map_path, '2', &sum_sqr_remission_map);
			carmen_grid_mapping_save_block_map_by_origin(map_path, 'c', &count_remission_map);

			//get new map with integrated information of the old map
			carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'm', x_origin, y_origin, &new_carmen_map);
			carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 's', x_origin, y_origin, &new_sum_remission_map);
			carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, '2', x_origin, y_origin, &new_sum_sqr_remission_map);
			carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'c', x_origin, y_origin, &new_count_remission_map);
		}
		else
		{
			carmen_grid_mapping_update_map_buffer(&map);
			carmen_grid_mapping_get_buffered_map(x_origin, y_origin, &new_carmen_map);
		}

		//destroy current map and assign new map to current map
		carmen_grid_mapping_swap_maps_and_clear_old_map(&map, &new_carmen_map);

		if (update_and_merge_with_mapper_saved_maps)
		{
			carmen_grid_mapping_swap_maps_and_clear_old_map(&sum_remission_map, &new_sum_remission_map);
			carmen_grid_mapping_swap_maps_and_clear_old_map(&sum_sqr_remission_map, &new_sum_sqr_remission_map);
			carmen_grid_mapping_swap_maps_and_clear_old_map(&count_remission_map, &new_count_remission_map);
		}
	}

	moving_objects_raw_map.config.x_origin = x_origin;
	moving_objects_raw_map.config.y_origin = y_origin;

	map.config.x_origin = x_origin;
	map.config.y_origin = y_origin;

	sum_remission_map.config.x_origin = x_origin;
	sum_remission_map.config.y_origin = y_origin;

	sum_sqr_remission_map.config.x_origin = x_origin;
	sum_sqr_remission_map.config.y_origin = y_origin;

	count_remission_map.config.x_origin = x_origin;
	count_remission_map.config.y_origin = y_origin;
}


static int
run_mapper(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global)
{
	//carmen_point_t world_pose;
	//carmen_position_t map_origin;

	if (!globalpos_initialized)
		return (0);

	//world_pose = globalpos_history[last_globalpos].globalpos;
	//carmen_grid_mapping_get_map_origin(&world_pose, &map_origin.x, &map_origin.y);

	build_map_using_velodyne(sensor_params, sensor_data, r_matrix_robot_to_global);
	
	return (1);
}


int
run_snapshot_mapper()
{
	int i;
	int current_point_cloud_index;//, before_point_cloud_index;
	static rotation_matrix *r_matrix_robot_to_global = NULL;
	static carmen_map_t *snapshot_map = NULL;
	
	snapshot_map = carmen_prob_models_check_if_new_snapshot_map_allocation_is_needed(snapshot_map, &map);
	
	if (!globalpos_initialized)
		return (0);

	if (sensors_params[0].alive)
	{
		current_point_cloud_index =  sensors_data[0].point_cloud_index;
//		before_point_cloud_index =  ((sensors_data[0].point_cloud_index - 1) + NUM_VELODYNE_POINT_CLOUDS) % NUM_VELODYNE_POINT_CLOUDS;

//		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_robot_to_global, sensors_data[0].robot_pose[before_point_cloud_index].orientation);
//		update_cells_in_the_velodyne_perceptual_field(snapshot_map, &sensors_params[0], &sensors_data[0], r_matrix_robot_to_global, before_point_cloud_index, UPDATE_CELLS_CROSSED_BY_RAYS);
//		carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(&map, snapshot_map);

		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_robot_to_global, sensors_data[0].robot_pose[current_point_cloud_index].orientation);
		update_cells_in_the_velodyne_perceptual_field( snapshot_map, &sensors_params[0], &sensors_data[0], r_matrix_robot_to_global, current_point_cloud_index, DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS, 0);
		carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(&map, snapshot_map);
	}

	for (i = 1; i < number_of_sensors; i++)
	{
		if (sensors_params[i].alive)
		{
			current_point_cloud_index =  sensors_data[i].point_cloud_index;
			r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_robot_to_global, sensors_data[i].robot_pose[current_point_cloud_index].orientation);
			update_cells_in_the_velodyne_perceptual_field(snapshot_map, &sensors_params[i], &sensors_data[i], r_matrix_robot_to_global, current_point_cloud_index, DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS, 0);
			carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(&map, snapshot_map);
		}
	}

	return (1);
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
mapper_velodyne_partial_scan(carmen_velodyne_partial_scan_message *velodyne_message)
{
	static int velodyne_message_id;
	//int ok_to_publish;

	int num_points = velodyne_message->number_of_32_laser_shots * sensors_params[0].vertical_resolution;

	ok_to_publish = 0;
	if (!globalpos_initialized)
		return (ok_to_publish);

	if (sensors_data[0].last_timestamp == 0.0)
	{
		sensors_data[0].last_timestamp = velodyne_message->timestamp;
		velodyne_message_id = -2;		// correntemente sao necessarias pelo menos 2 mensagens para ter uma volta completa de velodyne

		return (ok_to_publish);
	}
	
	sensors_data[0].current_timestamp = velodyne_message->timestamp;

	build_sensor_point_cloud(&sensors_data[0].points, sensors_data[0].intensity, &sensors_data[0].point_cloud_index, num_points, NUM_VELODYNE_POINT_CLOUDS);

	carmen_velodyne_partial_scan_update_points(velodyne_message, sensors_params[0].vertical_resolution,
			&sensors_data[0].points[sensors_data[0].point_cloud_index], sensors_data[0].intensity[sensors_data[0].point_cloud_index],
			sensors_params[0].ray_order,
			sensors_params[0].vertical_correction, sensors_params[0].range_max, velodyne_message->timestamp);

	sensors_data[0].robot_pose[sensors_data[0].point_cloud_index] = globalpos_history[last_globalpos].pose;
	sensors_data[0].robot_velocity[sensors_data[0].point_cloud_index] = globalpos_history[last_globalpos].velocity;
	sensors_data[0].robot_timestamp[sensors_data[0].point_cloud_index] = globalpos_history[last_globalpos].timestamp;
	sensors_data[0].robot_phi[sensors_data[0].point_cloud_index] = globalpos_history[last_globalpos].phi;


	if (velodyne_message_id >= 0)
	{
		if (build_snapshot_map)
			ok_to_publish = 1;
		else
			ok_to_publish = run_mapper(&sensors_params[0], &sensors_data[0], r_matrix_car_to_global);

		if (velodyne_message_id > 1000000)
			velodyne_message_id = 0;
	}
	velodyne_message_id++;
	sensors_data[0].last_timestamp = velodyne_message->timestamp;
	
	return (ok_to_publish);
}


int
mapper_velodyne_variable_scan(int sensor_number, carmen_velodyne_variable_scan_message *message)
{
	static int message_id;


	int num_points = message->number_of_shots * sensors_params[sensor_number].vertical_resolution;

	ok_to_publish = 0;
	if (!globalpos_initialized)
		return (ok_to_publish);

	if (sensors_data[sensor_number].last_timestamp == 0.0)
	{
		sensors_data[sensor_number].last_timestamp = message->timestamp;
		message_id = -2;		// correntemente sďż˝o necessďż˝rias pelo menos 2 mensagens para ter uma volta completa de velodyne

		return (ok_to_publish);
	}
	
	sensors_data[sensor_number].last_timestamp = sensors_data[sensor_number].current_timestamp = message->timestamp;

	build_sensor_point_cloud(&sensors_data[sensor_number].points, sensors_data[sensor_number].intensity, &sensors_data[sensor_number].point_cloud_index, num_points, NUM_VELODYNE_POINT_CLOUDS);

	carmen_velodyne_variable_scan_update_points(message, sensors_params[sensor_number].vertical_resolution,
			&sensors_data[sensor_number].points[sensors_data[sensor_number].point_cloud_index],
			sensors_data[sensor_number].intensity[sensors_data[sensor_number].point_cloud_index],
			sensors_params[sensor_number].ray_order, sensors_params[sensor_number].vertical_correction,
			sensors_params[sensor_number].range_max, message->timestamp);

	sensors_data[sensor_number].robot_pose[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].pose;
	sensors_data[sensor_number].robot_velocity[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].velocity;
	sensors_data[sensor_number].robot_timestamp[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].timestamp;
	sensors_data[sensor_number].robot_phi[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].phi;

	if (message_id >= 0)
	{
		if (build_snapshot_map)
			ok_to_publish = 1;
		else
			ok_to_publish = run_mapper(&sensors_params[sensor_number], &sensors_data[sensor_number], r_matrix_car_to_global);

		if (message_id > 1000000)
			message_id = 0;
	}
	message_id++;
	sensors_data[sensor_number].last_timestamp = message->timestamp;
	
	return (ok_to_publish);
}


void
mapper_merge_online_map_with_offline_map(carmen_map_t *offline_map)
{
	for (int i = 0; i < (map.config.x_size * map.config.y_size); i++)
		if (map.complete_map[i] < 0.0)
       			map.complete_map[i] = offline_map->complete_map[i];
}


void
mapper_publish_cost_map(double timestamp)
{
	carmen_compact_map_t compacted_cost_map;

	carmen_prob_models_build_obstacle_cost_map(&cost_map, &map,	map.config.resolution, obstacle_cost_distance, obstacle_probability_threshold);
	carmen_prob_models_create_compact_map(&compacted_cost_map, &cost_map, 0.0);

	if (compacted_cost_map.number_of_known_points_on_the_map > 0)
	{
		carmen_map_server_publish_compact_cost_map_message(&compacted_cost_map,	timestamp);
		carmen_prob_models_clear_carmen_map_using_compact_map(&cost_map, &compacted_cost_map, 0.0);
		carmen_prob_models_free_compact_map(&compacted_cost_map);
	}
}


void
mapper_publish_map(double timestamp)
{
	if (build_snapshot_map)
	{
		memcpy(map.complete_map, offline_map.complete_map, offline_map.config.x_size *  offline_map.config.y_size * sizeof(double));
		run_snapshot_mapper();
	}

	if (publish_moving_objects_raw_map) //Alberto: Estudo para fazer o modulo de DATMO
	{
		carmen_grid_mapping_moving_objects_raw_map_publish_message(&moving_objects_raw_map, timestamp);
		carmen_prob_models_clean_carmen_map(&moving_objects_raw_map);
	//	publish_frontlaser(timestamp);
	}

	carmen_grid_mapping_publish_message(&map, timestamp);

	mapper_publish_cost_map(timestamp);
}

void
mapper_set_robot_pose_into_the_map(carmen_localize_ackerman_globalpos_message *globalpos_message, int UPDATE_CELLS_BELOW_CAR)
{
	static double initial_time = 0.0;

	if (initial_time == 0.0)
		initial_time = carmen_get_time();
	
	if ((carmen_get_time() - initial_time) > 2.0)
		globalpos_initialized = 1;
	else
		return;

	last_globalpos = (last_globalpos + 1) % GLOBAL_POS_QUEUE_SIZE;

	globalpos_history[last_globalpos] = *globalpos_message;

	r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, globalpos_history[last_globalpos].pose.orientation);

	map.config.x_origin = x_origin;
	map.config.y_origin = y_origin;

	if (UPDATE_CELLS_BELOW_CAR)
		carmen_prob_models_updade_cells_bellow_robot(globalpos_message->globalpos, &map, 0.0, &car_config);
}


void
mapper_update_grid_map(carmen_point_t xt, double *zt, sensor_parameters_t *sensor_params)
{
	carmen_update_cells_in_the_sensor_perceptual_field(&map, xt, zt, sensor_params);
}


void
mapper_save_current_map()
{

	//save the current map in the given map_path
	carmen_grid_mapping_save_block_map_by_origin(map_path, 'm', &map);
	carmen_grid_mapping_save_block_map_by_origin(map_path, 's', &sum_remission_map);
	carmen_grid_mapping_save_block_map_by_origin(map_path, '2', &sum_sqr_remission_map);
	carmen_grid_mapping_save_block_map_by_origin(map_path, 'c', &count_remission_map);
}


void
mapper_initialize(carmen_map_config_t *main_map_config, carmen_robot_ackerman_config_t main_car_config)
{
	car_config = main_car_config;
	map_config = *main_map_config;
	
	carmen_grid_mapping_create_new_map(&map, map_config.x_size, map_config.y_size, map_config.resolution);
	carmen_grid_mapping_create_new_map(&offline_map, map_config.x_size, map_config.y_size, map_config.resolution);
	carmen_grid_mapping_create_new_map(&sum_remission_map, map_config.x_size, map_config.y_size, map_config.resolution);
	carmen_grid_mapping_create_new_map(&sum_sqr_remission_map, map_config.x_size, map_config.y_size, map_config.resolution);
	carmen_grid_mapping_create_new_map(&count_remission_map, map_config.x_size, map_config.y_size, map_config.resolution);

	globalpos_initialized = 0; // Only considered initialized when first message is received

	globalpos_history = (carmen_localize_ackerman_globalpos_message *) calloc(GLOBAL_POS_QUEUE_SIZE, sizeof(carmen_localize_ackerman_globalpos_message));

	last_globalpos = 0;
}

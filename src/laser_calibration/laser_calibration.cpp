#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <carmen/grid_mapping.h>

#include "laser_calibration.h"

#define	UPDATE_CELLS_CROSSED_BY_RAYS		1
#define	DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS	0

extern double safe_range_above_sensors;
extern double robot_wheel_radius;

extern double highest_sensor;

extern int merge_with_offline_map;
extern int update_and_merge_with_mapper_saved_maps;
extern int build_snapshot_map;

extern carmen_pose_3D_t sensor_board_1_pose;
extern rotation_matrix *board_to_car_matrix;

extern sensor_parameters_t sensors_params;
extern sensor_data_t sensors_data;

extern char *map_path;

double *sum_remission_intensity_matrix;
int *count_remission_matrix;
/**
 * The map
 */
carmen_map_t *remission_map, *sum_remission_map, *sum_sqr_remission_map, *count_remission_map;

extern carmen_map_t offline_map;

rotation_matrix *r_matrix_car_to_global = NULL;

int globalpos_initialized = 0;
carmen_localize_ackerman_globalpos_message *globalpos_history;
int last_globalpos;

carmen_robot_ackerman_config_t car_config;
carmen_map_config_t map_config;

double x_origin, y_origin; // map origin in meters



static void
update_cells_in_the_velodyne_perceptual_field(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global,
					      int point_cloud_index)
{
	int i, j;
	spherical_point_cloud v_zt = sensor_data->points[point_cloud_index];
	unsigned char *intensity = sensor_data->intensity[point_cloud_index];
	carmen_pose_3D_t robot_interpolated_position;

	double v = sensor_data->robot_velocity[point_cloud_index].x;
	double phi = sensor_data->robot_phi[point_cloud_index];

	double scan_time = (v_zt.num_points / sensor_params->vertical_resolution) * sensor_params->time_spent_by_each_scan;
	double dt = (sensor_data->current_timestamp - sensor_data->robot_timestamp[point_cloud_index]) - scan_time;
	sensor_params->current_range_max = sensor_params->range_max;

	// Ray-trace the grid
	for (i = 0, j = 0; i < v_zt.num_points; i = i +  sensor_params->vertical_resolution, j++)
	{
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(sensor_data->robot_pose[point_cloud_index], dt, v, phi, car_config.distance_between_front_and_rear_axles);
		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);
		// O angulo do Velodyne ee enviado no sentido horario, ao inves de anti-horario como pede Carmen

		carmen_prob_models_compute_relevant_map_coordinates(sensor_params, sensor_data, &(v_zt.sphere_points[i]), &intensity[i], robot_interpolated_position.position, sensor_board_1_pose,
				r_matrix_robot_to_global, board_to_car_matrix, robot_wheel_radius, x_origin, y_origin, car_config.distance_between_rear_car_and_rear_wheels,
				car_config.length, car_config.width);

		carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(sensor_params, sensor_data,
				highest_sensor, safe_range_above_sensors, sensor_params->delta_difference_mean, sensor_params->delta_difference_stddev);

		carmen_prob_models_update_intensity_of_cells_hit_by_rays_for_calibration(sum_remission_map, sum_sqr_remission_map, count_remission_map, remission_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors);
	}
}




static void
build_map_using_velodyne(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global)
{
	update_cells_in_the_velodyne_perceptual_field(sensor_params, sensor_data, r_matrix_robot_to_global, sensor_data->point_cloud_index);
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
			carmen_grid_mapping_initialize_map(current_carmen_map, remission_map[0].config.x_size, remission_map[0].config.resolution);
	}
	else		
		carmen_grid_mapping_get_buffered_map(x_origin, y_origin, current_carmen_map);
}



void
laser_calibration(carmen_position_t *map_origin)
{
	int i,  j;
	FILE *f;
	static int  first_time = 1;

	x_origin = map_origin->x;
	y_origin = map_origin->y;

	for(i = 0; i < sensors_params.vertical_resolution; i++)
	{
		remission_map[i].config.x_origin = x_origin;
		remission_map[i].config.y_origin = y_origin;
		sum_remission_map[i].config.x_origin = x_origin;
		sum_remission_map[i].config.y_origin = y_origin;
		count_remission_map[i].config.x_origin = x_origin;
		count_remission_map[i].config.y_origin = y_origin;
		sum_sqr_remission_map[i].config.x_origin = x_origin;
		sum_sqr_remission_map[i].config.y_origin = y_origin;
	}

	if (first_time)
	{
		first_time = 0;
		return;
	}

	f = fopen("remission_calibration.txt", "w");

	for(i = 0; i < sensors_params.vertical_resolution; i++)
	{
		for(j = 0; j < (remission_map[i].config.x_size * remission_map[i].config.y_size); j++)
		{
			if(remission_map[i].complete_map[j] >= 0.0 && count_remission_map[i].complete_map[j] >= 0.0)
			{
				sum_remission_intensity_matrix[i * 256 + (int) remission_map[i].complete_map[j]] += sum_remission_map[i].complete_map[j];
				count_remission_matrix[i * 256 + (int) remission_map[i].complete_map[j]] += count_remission_map[i].complete_map[j];
			}
			sum_remission_map[i].complete_map[j] = -1.0;
			count_remission_map[i].complete_map[j] = -1.0;
			remission_map[i].complete_map[j] = -1.0;
			sum_sqr_remission_map[i].complete_map[j] = -1.0;
		}
	}
	for(i = 0; i < sensors_params.vertical_resolution; i++)
	{
		for (j = 0; j < 256; j++)
		{
			if (count_remission_matrix[i * 256 + j] > 0.0)
				fprintf(f, "%lf\n",(sum_remission_intensity_matrix[i * 256 + j] / (double)count_remission_matrix[i * 256 + j]));
			else
				fprintf(f, "%lf\n", (double)j / 256.0);

		}
	}

	fclose(f);
}


void
laser_calibration_using_map(carmen_localize_ackerman_map_t *map)
{
	int i,  j;
	FILE *f;
	static int  first_time = 1;

	x_origin = map->config.x_origin;
	y_origin = map->config.y_origin;

	for(i = 0; i < sensors_params.vertical_resolution; i++)
	{
		remission_map[i].config.x_origin = map->config.x_origin;
		remission_map[i].config.y_origin = map->config.y_origin;
		sum_remission_map[i].config.x_origin = map->config.x_origin;
		sum_remission_map[i].config.y_origin = map->config.y_origin;
		count_remission_map[i].config.x_origin = map->config.x_origin;
		count_remission_map[i].config.y_origin = map->config.y_origin;
		sum_sqr_remission_map[i].config.x_origin = map->config.x_origin;
		sum_sqr_remission_map[i].config.y_origin = map->config.y_origin;
	}

	if (first_time)
	{
		first_time = 0;
		return;
	}

	f = fopen("remission_calibration_using_map.txt", "w");

	for(i = 0; i < sensors_params.vertical_resolution; i++)
	{
		for(j = 0; j < (map->config.x_size * map->config.y_size); j++)
		{
			if(map->carmen_map.complete_map[j] >= 0.3)
			{
				sum_remission_intensity_matrix[i * 256 + (int) remission_map[i].complete_map[j]] += map->carmen_mean_remission_map.complete_map[j];
				count_remission_matrix[i * 256 + (int) remission_map[i].complete_map[j]]++;
			}
			sum_remission_map[i].complete_map[j] = -1.0;
			count_remission_map[i].complete_map[j] = -1.0;
			remission_map[i].complete_map[j] = -1.0;
			sum_sqr_remission_map[i].complete_map[j] = -1.0;
		}
	}
	for(i = 0; i < sensors_params.vertical_resolution; i++)
	{
		for (j = 0; j < 256; j++)
		{
			if (count_remission_matrix[i * 256 + j] > 0.0)
				fprintf(f, "%lf\n",(sum_remission_intensity_matrix[i * 256 + j] / (double)count_remission_matrix[i * 256 + j]));
			else
				fprintf(f, "%lf\n", 0.0);

		}
	}

	fclose(f);
}



static int
run_mapper(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global)
{
	carmen_point_t world_pose;
	//carmen_position_t map_origin;

	if (!globalpos_initialized)
		return 0;

	world_pose = globalpos_history[last_globalpos].globalpos;
	//carmen_grid_mapping_get_map_origin(&world_pose, &map_origin.x, &map_origin.y);

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
mapper_velodyne_partial_scan(carmen_velodyne_partial_scan_message *velodyne_message)
{
	static int velodyne_message_id;

	int num_points = velodyne_message->number_of_32_laser_shots * sensors_params.vertical_resolution;

	if (!globalpos_initialized)
		return 1;

	if (sensors_data.last_timestamp == 0.0)
	{
		sensors_data.last_timestamp = velodyne_message->timestamp;
		velodyne_message_id = -2;		// correntemente sďż˝o necessďż˝rias pelo menos 2 mensagens para ter uma volta completa de velodyne

		return 1;
	}
	
	sensors_data.current_timestamp = velodyne_message->timestamp;

	build_sensor_point_cloud(&sensors_data.points, sensors_data.intensity, &sensors_data.point_cloud_index, num_points, NUM_VELODYNE_POINT_CLOUDS);

	carmen_velodyne_partial_scan_update_points(velodyne_message, sensors_params.vertical_resolution,
			&sensors_data.points[sensors_data.point_cloud_index], sensors_data.intensity[sensors_data.point_cloud_index],
			sensors_params.ray_order,
			sensors_params.vertical_correction, sensors_params.range_max);

	sensors_data.robot_pose[sensors_data.point_cloud_index] = globalpos_history[last_globalpos].pose;
	sensors_data.robot_velocity[sensors_data.point_cloud_index] = globalpos_history[last_globalpos].velocity;
	sensors_data.robot_timestamp[sensors_data.point_cloud_index] = globalpos_history[last_globalpos].timestamp;
	sensors_data.robot_phi[sensors_data.point_cloud_index] = globalpos_history[last_globalpos].phi;

	if (velodyne_message_id >= 0)
	{
		run_mapper(&sensors_params, &sensors_data, r_matrix_car_to_global);

		if (velodyne_message_id > 1000000)
			velodyne_message_id = 0;
	}
	velodyne_message_id++;
	sensors_data.last_timestamp = velodyne_message->timestamp;
	
	return 1;
}

void
mapper_set_robot_pose_into_the_map(carmen_localize_ackerman_globalpos_message *globalpos_message)
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
}

void
mapper_save_current_map()
{
//
//	//save the current map in the given map_path
//	carmen_grid_mapping_save_block_map_by_origin(map_path, 'm', &map);
//	carmen_grid_mapping_save_block_map_by_origin(map_path, 's', &sum_remission_map);
//	carmen_grid_mapping_save_block_map_by_origin(map_path, '2', &sum_sqr_remission_map);
//	carmen_grid_mapping_save_block_map_by_origin(map_path, 'c', &count_remission_map);
}


void
mapper_initialize(carmen_map_config_t *main_map_config, carmen_robot_ackerman_config_t main_car_config, int vertical_resolution)
{
	int i;
	car_config = main_car_config;
	map_config = *main_map_config;
	
	remission_map = (carmen_map_t*)calloc(sizeof(carmen_map_t), vertical_resolution);
	sum_remission_map = (carmen_map_t*)calloc(sizeof(carmen_map_t), vertical_resolution);
	sum_sqr_remission_map = (carmen_map_t*)calloc(sizeof(carmen_map_t), vertical_resolution);
	count_remission_map = (carmen_map_t*)calloc(sizeof(carmen_map_t), vertical_resolution);
	sum_remission_intensity_matrix = (double*)calloc(sizeof(double), 256 * vertical_resolution);
	count_remission_matrix = (int*)calloc(sizeof(int), 256 * vertical_resolution);

	for(i = 0; i < vertical_resolution; i ++)
	{
		carmen_grid_mapping_initialize_map(&remission_map[i], map_config.x_size, map_config.resolution);
		carmen_grid_mapping_initialize_map(&sum_remission_map[i], map_config.x_size, map_config.resolution);
		carmen_grid_mapping_initialize_map(&sum_sqr_remission_map[i], map_config.x_size, map_config.resolution);
		carmen_grid_mapping_initialize_map(&count_remission_map[i], map_config.x_size, map_config.resolution);
	}
	globalpos_initialized = 0; // Only considered initialized when first message is received

	globalpos_history = (carmen_localize_ackerman_globalpos_message *) calloc(GLOBAL_POS_QUEUE_SIZE, sizeof(carmen_localize_ackerman_globalpos_message));

	last_globalpos = 0;
}


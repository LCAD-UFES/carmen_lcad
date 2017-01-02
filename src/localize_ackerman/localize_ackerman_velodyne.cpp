#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_map.h>
#include "localize_ackerman_velodyne.h"
#include "localize_ackerman_core.h"

#include <carmen/stereo_velodyne.h>
#include <carmen/stereo_velodyne_interface.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

typedef struct
{
	double max_range;
	double variance;
	double prob_of_random_reading;
	double prob_of_random_max;
	int num_lasers;
	double offset;
	double side_offset;
	double angular_offset;
	double angular_resolution;
	double fov;
	double start_angle;
	int id;
} localize_ackerman_velodyne_laser_config_t;


extern double safe_range_above_sensors;

extern carmen_pose_3D_t sensor_board_1_pose;
extern rotation_matrix *sensor_board_1_to_car_matrix;

extern double robot_wheel_radius;
extern double highest_sensor;
extern carmen_robot_ackerman_config_t car_config;

extern double x_origin;
extern double y_origin;

extern carmen_map_t local_map;
extern carmen_map_t local_sum_remission_map;
extern carmen_map_t local_mean_remission_map;
extern carmen_map_t local_variance_remission_map;
extern carmen_map_t local_sum_sqr_remission_map;
extern carmen_map_t local_count_remission_map;

extern carmen_compact_map_t local_compacted_map;
extern carmen_compact_map_t local_compacted_mean_remission_map;
extern carmen_compact_map_t local_compacted_variance_remission_map;
extern carmen_localize_ackerman_binary_map_t binary_map;

extern int velodyne_viewer;

double laser_ranges[10000];

extern carmen_localize_ackerman_particle_filter_p filter;
static localize_ackerman_velodyne_laser_config_t front_laser_config;
static carmen_laser_laser_message flaser_message;

IplImage *map_image = NULL;

extern cell_coords_t **map_cells_hit_by_each_rays;
extern int number_of_threads;

static void
localize_ackerman_velodyne_laser_initialize()
{
	flaser_message.host = carmen_get_host();
	flaser_message.num_readings = front_laser_config.num_lasers;
	flaser_message.range = (double *) calloc(front_laser_config.num_lasers, sizeof(double));
	carmen_test_alloc(flaser_message.range);

	flaser_message.num_remissions = 0;
	flaser_message.remission = 0;

	flaser_message.id = front_laser_config.id;
	flaser_message.num_readings = front_laser_config.num_lasers;

	flaser_message.config.maximum_range       = front_laser_config.max_range;
	flaser_message.config.fov                 = front_laser_config.fov;
	flaser_message.config.start_angle         = front_laser_config.start_angle;
	flaser_message.config.angular_resolution  = front_laser_config.angular_resolution;

	//this was placed here because compiling with the old motion model
	//did't work, check this if this breaks something
	flaser_message.config.remission_mode      = REMISSION_NONE;
}


static void
fill_laser_config_data(localize_ackerman_velodyne_laser_config_t *lasercfg)
{
	lasercfg->num_lasers = 1 + carmen_round(lasercfg->fov / lasercfg->angular_resolution);
	lasercfg->start_angle = -0.5 * lasercfg->fov;

	/* give a warning if it is not a standard configuration */

	if (fabs(lasercfg->fov - M_PI) > 1e-6 &&
			fabs(lasercfg->fov - 100.0/180.0 * M_PI) > 1e-6 &&
			fabs(lasercfg->fov -  90.0/180.0 * M_PI) > 1e-6)
		carmen_warn("Warning: You are not using a standard SICK configuration (fov=%.4f deg)\n",
				carmen_radians_to_degrees(lasercfg->fov));

	if (fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(1.0)) > 1e-6 &&
			fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.5)) > 1e-6 &&
			fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.25)) > 1e-6)
		carmen_warn("Warning: You are not using a standard SICK configuration (res=%.4f deg)\n",
				carmen_radians_to_degrees(lasercfg->angular_resolution));

}


void
localize_ackerman_velodyne_laser_read_parameters(int argc, char **argv)
{
	static char frontlaser_fov_string[256];
	static char frontlaser_res_string[256];

	carmen_param_t param_list[] =
	{
			{(char *)"robot", (char*)"frontlaser_id", CARMEN_PARAM_INT, &(front_laser_config.id), 0, NULL}
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	sprintf(frontlaser_fov_string, (char*)"laser%d_fov", front_laser_config.id);
	sprintf(frontlaser_res_string, (char*)"laser%d_resolution", front_laser_config.id);

	carmen_param_t param_list_front_laser[] =
	{
			{(char *)"simulator", (char*)"frontlaser_maxrange", CARMEN_PARAM_DOUBLE, &(front_laser_config.max_range), 1, NULL},
			{(char *)"robot", (char*)"frontlaser_offset", CARMEN_PARAM_DOUBLE, &(front_laser_config.offset), 1, NULL},
			{(char *)"robot", (char*)"frontlaser_side_offset", CARMEN_PARAM_DOUBLE, &(front_laser_config.side_offset), 1, NULL},
			{(char *)"robot", (char*)"frontlaser_angular_offset", CARMEN_PARAM_DOUBLE, &(front_laser_config.angular_offset), 1, NULL},
			{(char *)"laser", frontlaser_fov_string, CARMEN_PARAM_DOUBLE, &(front_laser_config.fov), 0, NULL},
			{(char *)"laser", frontlaser_res_string, CARMEN_PARAM_DOUBLE, &(front_laser_config.angular_resolution), 0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list_front_laser, sizeof(param_list_front_laser) / sizeof(param_list_front_laser[0]));

	front_laser_config.angular_resolution =	carmen_degrees_to_radians(front_laser_config.angular_resolution);

	front_laser_config.fov = carmen_degrees_to_radians(front_laser_config.fov);

	fill_laser_config_data(&front_laser_config);

	localize_ackerman_velodyne_laser_initialize();
}


static void
calc_velodyne_laser(double *laser_ranges)
{
	int index;

	for (int i = 0; i < flaser_message.config.fov / flaser_message.config.angular_resolution; i++)
	{
		index = (int) (((flaser_message.config.start_angle - front_laser_config.angular_offset) / flaser_message.config.angular_resolution) +
				(M_PI / flaser_message.config.angular_resolution) + 0.5) + i;

		if (index < 0)
			index = index + ((2.0 * M_PI) / flaser_message.config.angular_resolution);
		else if (index > ((2.0 * M_PI) / flaser_message.config.angular_resolution))
			index = index - ((2.0 * M_PI) / flaser_message.config.angular_resolution);
		flaser_message.range[i] = laser_ranges[index];
	}
}

carmen_laser_laser_message *
localize_ackerman_velodyne_create_frontlaser_message(double timestamp, double *laser_ranges)
{
	calc_velodyne_laser(laser_ranges);

	flaser_message.timestamp = timestamp;

	return &flaser_message;
}


void
localize_ackerman_velodyne_publish_frontlaser(double timestamp, double *laser_ranges)
{
	IPC_RETURN_TYPE err = IPC_OK;

	localize_ackerman_velodyne_create_frontlaser_message(timestamp, laser_ranges);

	err = IPC_publishData(CARMEN_LASER_FRONTLASER_NAME, &flaser_message);
	carmen_test_ipc(err, "Could not publish laser_frontlaser_message",
			CARMEN_LASER_FRONTLASER_NAME);
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
get_the_laser_ray_angle_index_from_angle(double angle)
{
	double angle_from_origin;
	int index;

	angle_from_origin = angle + M_PI;
	index = (int) ((angle_from_origin / front_laser_config.angular_resolution) + 0.5);

	return (index);
}


void
save_local_map()
{
	carmen_FILE *fp;
	fp = carmen_fopen("local.map", "w");
	carmen_map_write_all(fp, local_map.map,
				local_map.config.x_size,
				local_map.config.y_size,
				local_map.config.resolution,
				(char *) "", (char *) "", (char *) "", (char *) "Generated by big_map",
				NULL, 0, NULL, 0, NULL, 0);
	carmen_fclose(fp);
}


static void
compute_laser_rays_from_velodyne_and_create_a_local_map(sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data, rotation_matrix *r_matrix_car_to_global,
				 carmen_pose_3D_t *robot_pose, carmen_vector_3D_t *robot_velocity,
				 double x_origin, double y_origin, int point_cloud_index, double phi)
{
	spherical_point_cloud v_zt = velodyne_data->points[point_cloud_index];
	int N = v_zt.num_points / velodyne_params->vertical_resolution;

	double dt = velodyne_params->time_spent_by_each_scan;
	double dt1 = -(double) N * dt;
	carmen_pose_3D_t robot_interpolated_position = *robot_pose;

	// Ray-trace the grid
	int jump = filter->param->jump_size;
	for (int j = 0; j < N; j += jump)
	{
		double dt2 = j * dt;
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(*robot_pose, dt1 + dt2, robot_velocity->x, phi,
				car_config.distance_between_front_and_rear_axles);
		r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

		carmen_prob_models_compute_relevant_map_coordinates(velodyne_data, velodyne_params, j * velodyne_params->vertical_resolution,
				robot_interpolated_position.position, sensor_board_1_pose, r_matrix_car_to_global, sensor_board_1_to_car_matrix,
				robot_wheel_radius, x_origin, y_origin, &car_config, 0, 0);

		carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(velodyne_data, velodyne_params, j * velodyne_params->vertical_resolution, highest_sensor, safe_range_above_sensors, 0, 0);

		carmen_prob_models_upgrade_log_odds_of_cells_hit_by_rays(&local_map, velodyne_params, velodyne_data, 0);

		carmen_prob_models_update_intensity_of_cells_hit_by_rays(&local_sum_remission_map, &local_sum_sqr_remission_map, &local_count_remission_map, velodyne_params, velodyne_data, highest_sensor, safe_range_above_sensors, NULL, 0);
	}
}


void
create_rand_vector(int **rand_vector_in, int vector_size, int number_of_measurement)
{
	int i;
	srand(time(NULL));

	int *rand_vector = *rand_vector_in;

	if (rand_vector != NULL)
		free(rand_vector);
	rand_vector = (int *)malloc(vector_size * sizeof(int));

	for (i = 0; i < vector_size; i++)
	{
		rand_vector[i] = i;
	}

	for (i = 0; i < vector_size; i++)
	{
		int r1 = (int)(((double)rand() / RAND_MAX) * number_of_measurement);
		int r2 = (int)(((double)rand() / RAND_MAX) * number_of_measurement);

		int aux = rand_vector[r1];
		rand_vector[r1] = rand_vector[r2];
		rand_vector[r2] = aux;
	}
	*rand_vector_in =  rand_vector;
}


void
create_binary_map(carmen_localize_ackerman_binary_map_t *map, carmen_compact_map_t compact_map)
{
	int i;
	create_rand_vector(&map->rand_position, compact_map.number_of_known_points_on_the_map, compact_map.number_of_known_points_on_the_map);
	map->map_size = compact_map.number_of_known_points_on_the_map;

	if (map->binary_map != NULL)
		free(map->binary_map);

	map->binary_map = (int *)malloc(map->map_size * sizeof(int));

	for (i = 0; i < map->map_size - 1; i++)
	{
		map->binary_map[i] = compact_map.value[map->rand_position[i]] > compact_map.value[map->rand_position[i + 1]] ? 1 : 0;
	}
	map->binary_map[i] = compact_map.value[map->rand_position[i]] > compact_map.value[map->rand_position[0]] ? 1 : 0;
}


void
equalize_image2(IplImage *img)
{
	IplImage *img_gray = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);

	int i, j;
	for (j = 0; j < img->height; j++)
	{
		for (i = 0; i < img->width; i++)
		{
			if ((unsigned char)img->imageData[j * img->widthStep + 3 * i] == 255 &&
				(unsigned char)img->imageData[j * img->widthStep + 3 * i + 1] == 0 &&
				(unsigned char)img->imageData[j * img->widthStep + 3 * i + 2] == 0)
				img_gray->imageData[j * img_gray->widthStep + i] = 255;
			else
				img_gray->imageData[j * img_gray->widthStep + i] = img->imageData[j * img->widthStep + 3 * i];
		}
	}
	cvEqualizeHist(img_gray,img_gray);

	for (j = 0; j < img->height; j++)
	{
		for (i = 0; i < img->width; i++)
		{
			if ((unsigned char)img->imageData[j * img->widthStep + 3 * i] == 255 &&
				(unsigned char)img->imageData[j * img->widthStep + 3 * i + 1] == 0 &&
				(unsigned char)img->imageData[j * img->widthStep + 3 * i + 2] == 0)
				img->imageData[j * img->widthStep + 3 * i] = 255;
			else
			{
				img->imageData[j * img->widthStep + 3 * i] = img_gray->imageData[j * img_gray->widthStep + i];
				img->imageData[j * img->widthStep + 3 * i + 1] = img_gray->imageData[j * img_gray->widthStep + i];
				img->imageData[j * img->widthStep + 3 * i + 2] = img_gray->imageData[j * img_gray->widthStep + i];
			}
		}
	}

	cvReleaseImage(&img_gray);
}


void
debug_remission_map_velodyne(carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *velodyne_params)
{
	int k, r, l;
	int i, j;
	static IplImage *range_image, *remission_image;
	static char *cell_touched;

	if (map_image == NULL)
	{
		map_image = cvCreateImage(cvSize(2 * 360, velodyne_params->vertical_resolution * 2), IPL_DEPTH_8U, 3);
		range_image = cvCreateImage(cvSize(2 * 360, velodyne_params->vertical_resolution * 2), IPL_DEPTH_8U, 3);
		remission_image = cvCreateImage(cvSize(2 * 360, velodyne_params->vertical_resolution * 2), IPL_DEPTH_8U, 3);

		map_cells_hit_by_each_rays = (cell_coords_t **)malloc((velodyne_message->number_of_32_laser_shots + 50) * sizeof(cell_coords_t *));
		cell_touched = (char *)calloc(local_mean_remission_map.config.x_size * local_mean_remission_map.config.y_size, sizeof(char));

		for (i = 0; i < velodyne_message->number_of_32_laser_shots + 50; i++)
		{
			map_cells_hit_by_each_rays[i] = (cell_coords_t *)calloc(velodyne_params->vertical_resolution, sizeof(cell_coords_t));
		}
	}

	memset(cell_touched, 0, local_mean_remission_map.config.x_size * local_mean_remission_map.config.y_size * sizeof(char));
	for (l = 0, j = 0; j < velodyne_params->vertical_resolution; j++, l+=2)
	{
		k = 0;
		r = velodyne_params->ray_order[(velodyne_params->vertical_resolution - 1) - j];
		for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
		{
			k = i;//(k + 1) % velodyne_message->number_of_32_laser_shots;
			int index = ((int)((velodyne_message->partial_scan[i].angle + 180.0) / 0.5) % 720);

			if ((velodyne_message->partial_scan[k].distance[r]) != 0)
			{
				float range = (velodyne_message->partial_scan[k].distance[r] / 500.0);
				if (range < 50.0)
				{
					cell_coords_t map_cell = map_cells_hit_by_each_rays[k][r];

					range_image->imageData[l * range_image->widthStep + 3 * index] = 255 * (range / 50.0);
					range_image->imageData[l * range_image->widthStep + 3 * index + 1] = 255 * (range / 50.0);
					range_image->imageData[l * range_image->widthStep + 3 * index + 2] = 255 * (range / 50.0);
					if (/*!cell_touched[map_cell.x * local_mean_remission_map.config.x_size + map_cell.y]*/1)
					{
						cell_touched[map_cell.x * local_mean_remission_map.config.x_size + map_cell.y] = 1;
						map_image->imageData[l * map_image->widthStep + 3 * index] =  255 * local_mean_remission_map.map[map_cell.x][map_cell.y];//velodyne_message->partial_scan[k].intensity[r];
						map_image->imageData[l * map_image->widthStep + 3 * index + 1] =  255 * local_mean_remission_map.map[map_cell.x][map_cell.y];//velodyne_message->partial_scan[k].intensity[r];
						map_image->imageData[l * map_image->widthStep + 3 * index + 2] =  255 * local_mean_remission_map.map[map_cell.x][map_cell.y];//velodyne_message->partial_scan[k].intensity[r];
					}
					else
						map_image->imageData[l * map_image->widthStep + 3 * index] =  255;

					remission_image->imageData[l * remission_image->widthStep + 3 * index] = velodyne_message->partial_scan[k].intensity[r];
					remission_image->imageData[l * remission_image->widthStep + 3 * index + 1] = velodyne_message->partial_scan[k].intensity[r];
					remission_image->imageData[l * remission_image->widthStep + 3 * index + 2] = velodyne_message->partial_scan[k].intensity[r];
				}
				else
				{
					range_image->imageData[l * range_image->widthStep + 3 * index] = 255;
					map_image->imageData[l * map_image->widthStep + 3 * index] = 255;
					remission_image->imageData[l * remission_image->widthStep + 3 * index] = 255;
				}
			}
			else
			{
				range_image->imageData[l * range_image->widthStep + 3 * index] = 255;
				map_image->imageData[l * map_image->widthStep + 3 * index] = 255;
				remission_image->imageData[l * remission_image->widthStep + 3 * index] = 255;
			}

		}

	}

	for (l = 0, j = 0; j < velodyne_params->vertical_resolution; j++, l+=2)
	{
		memcpy(&range_image->imageData[(l + 1) * range_image->widthStep], &range_image->imageData[l * range_image->widthStep], range_image->widthStep);
		//memcpy(&range_image->imageData[(l + 2) * velodyne_message->number_of_32_laser_shots], &range_image->imageData[l * velodyne_message->number_of_32_laser_shots], velodyne_message->number_of_32_laser_shots);
		//memcpy(&range_image->imageData[(l + 3) * velodyne_message->number_of_32_laser_shots], &range_image->imageData[l * velodyne_message->number_of_32_laser_shots], velodyne_message->number_of_32_laser_shots);

		memcpy(&map_image->imageData[(l + 1) * map_image->widthStep], &map_image->imageData[l * map_image->widthStep], map_image->widthStep);
		memcpy(&remission_image->imageData[(l + 1) * remission_image->widthStep], &remission_image->imageData[l * remission_image->widthStep], remission_image->widthStep);
		//memcpy(&map_image->imageData[(l + 2) * velodyne_message->number_of_32_laser_shots], &map_image->imageData[l * velodyne_message->number_of_32_laser_shots], velodyne_message->number_of_32_laser_shots);
		//memcpy(&map_image->imageData[(l + 3) * velodyne_message->number_of_32_laser_shots], &map_image->imageData[l * velodyne_message->number_of_32_laser_shots], velodyne_message->number_of_32_laser_shots);
		//memset(map_cells_hit_by_each_rays, 0, sensor_params->vertical_resolution * sizeof(cell_coords_t));
	}


	//equalize_image(remission_image);
	//equalize_image(range_image);
	cvShowImage("remission_image", remission_image);
	cvShowImage("range_image", range_image);
	cvShowImage("map_image", map_image);
	cvWaitKey(33);
}


// Computes local_compacted_map and local_compacted_mean_remission_map maps
int
localize_ackerman_velodyne_partial_scan(carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data,
					carmen_vector_3D_t *robot_velocity, double phi)
{
	static rotation_matrix *r_matrix_car_to_global = NULL;
	static int velodyne_message_id;
	int current_point_cloud_index;

	int num_points = velodyne_message->number_of_32_laser_shots * velodyne_params->vertical_resolution;

	if (velodyne_data->last_timestamp == 0.0)
	{
		velodyne_data->last_timestamp = velodyne_message->timestamp;
		velodyne_message_id = -2; // correntemente sao necessarias pelo menos 2 mensagens para se ter uma volta completa de velodyne
		return (0);
	}
	
	velodyne_data->current_timestamp = velodyne_message->timestamp;

	build_sensor_point_cloud(&(velodyne_data->points), velodyne_data->intensity, &(velodyne_data->point_cloud_index), num_points, NUM_VELODYNE_POINT_CLOUDS);

	carmen_velodyne_partial_scan_update_points(velodyne_message,
			velodyne_params->vertical_resolution,
			&(velodyne_data->points[velodyne_data->point_cloud_index]),
			velodyne_data->intensity[velodyne_data->point_cloud_index],
			velodyne_params->ray_order,
			velodyne_params->vertical_correction,
			velodyne_params->range_max,
			velodyne_message->timestamp);

	//if (velodyne_viewer)
		//debug_remission_map_velodyne(velodyne_message, velodyne_params);
	
	if (velodyne_message_id >= 0)
	{
		carmen_pose_3D_t local_pose;

		local_pose.position.x = (local_map.config.x_size * local_map.config.resolution) / 2.0;
		local_pose.position.y = (local_map.config.x_size * local_map.config.resolution) / 2.0;
		local_pose.position.z = 0;
		local_pose.orientation.pitch = local_pose.orientation.roll = local_pose.orientation.yaw = 0.0;

		r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, local_pose.orientation);

		current_point_cloud_index =  velodyne_data->point_cloud_index;

		compute_laser_rays_from_velodyne_and_create_a_local_map(velodyne_params, velodyne_data, r_matrix_car_to_global, &local_pose, robot_velocity, 0.0, 0.0, current_point_cloud_index, phi);

		carmen_prob_models_calc_mean_and_variance_remission_map(&local_mean_remission_map, &local_variance_remission_map,
							&local_sum_remission_map, &local_sum_sqr_remission_map, &local_count_remission_map);

		carmen_prob_models_free_compact_map(&local_compacted_map);
		carmen_prob_models_free_compact_map(&local_compacted_mean_remission_map);
//		carmen_prob_models_free_compact_map(&local_compacted_variance_remission_map);

		// Build local_compacted_map from local_map computed by compute_laser_rays_from_velodyne_and_create_a_local_map()
		carmen_prob_models_create_compact_map(&local_compacted_map, &local_map, -1.0);
		carmen_prob_models_create_compact_map(&local_compacted_mean_remission_map, &local_mean_remission_map, -1.0);
//		carmen_prob_models_create_compact_map(&local_compacted_variance_remission_map, &local_variance_remission_map, -1.0);

		// Clear maps for next point cloud
		carmen_prob_models_clear_carmen_map_using_compact_map(&local_map, &local_compacted_map, -1.0);
		carmen_prob_models_clear_carmen_map_using_compact_map(&local_mean_remission_map, &local_compacted_mean_remission_map, -1.0);
//		carmen_prob_models_clear_carmen_map_using_compact_map(&local_variance_remission_map, &local_compacted_variance_remission_map, -1.0);
		carmen_prob_models_clear_carmen_map_using_compact_map(&local_sum_remission_map, &local_compacted_mean_remission_map, -1.0);
//		carmen_prob_models_clear_carmen_map_using_compact_map(&local_sum_sqr_remission_map, &local_compacted_variance_remission_map, -1.0);
		carmen_prob_models_clear_carmen_map_using_compact_map(&local_count_remission_map, &local_compacted_mean_remission_map, -1.0);

		if (velodyne_message_id > 1000000)
			velodyne_message_id = 0;
	}
	velodyne_message_id++;
	velodyne_data->last_timestamp = velodyne_message->timestamp;

	if (velodyne_message_id >= 0)
		return (1);
	else
		return (0);
}


int
localize_ackerman_velodyne_variable_scan(carmen_velodyne_variable_scan_message *message, sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data,
					carmen_vector_3D_t *robot_velocity)
{
	static rotation_matrix *r_matrix_car_to_global = NULL;
	static int message_id;
	int current_point_cloud_index;

	int num_points = message->number_of_shots * velodyne_params->vertical_resolution;

	if (velodyne_data->last_timestamp == 0.0)
	{
		velodyne_data->last_timestamp = message->timestamp;
		message_id = -2;		// correntemente sao necessarias pelo menos 2 mensagens para se ter uma volta completa de velodyne

		return (0);
	}

	velodyne_data->current_timestamp = message->timestamp;

	build_sensor_point_cloud(&(velodyne_data->points), velodyne_data->intensity, &(velodyne_data->point_cloud_index), num_points, NUM_VELODYNE_POINT_CLOUDS);

	carmen_velodyne_variable_scan_update_points(message,
			velodyne_params->vertical_resolution,
			&(velodyne_data->points[velodyne_data->point_cloud_index]),
			velodyne_data->intensity[velodyne_data->point_cloud_index],
			velodyne_params->ray_order,
			velodyne_params->vertical_correction,
			velodyne_params->range_max,
			message->timestamp);

	if (message_id >= 0)
	{
		carmen_pose_3D_t local_pose;

		local_pose.position.x = (local_map.config.x_size * local_map.config.resolution) / 2.0;
		local_pose.position.y = (local_map.config.x_size * local_map.config.resolution) / 2.0;
		local_pose.position.z = 0;
		local_pose.orientation.pitch = local_pose.orientation.roll = local_pose.orientation.yaw = 0.0;

		r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, local_pose.orientation);

		current_point_cloud_index =  velodyne_data->point_cloud_index;

		compute_laser_rays_from_velodyne_and_create_a_local_map(velodyne_params, velodyne_data, r_matrix_car_to_global, &local_pose, robot_velocity, 0.0, 0.0, current_point_cloud_index, 0.0);

//		carmen_prob_models_free_compact_map(&local_compacted_map);
//		carmen_prob_models_create_compact_map(&local_compacted_map, &local_map);

		carmen_prob_models_calc_mean_and_variance_remission_map(&local_mean_remission_map, &local_variance_remission_map,
				&local_sum_remission_map, &local_sum_sqr_remission_map, &local_count_remission_map);

		carmen_prob_models_free_compact_map(&local_compacted_map);
		carmen_prob_models_free_compact_map(&local_compacted_mean_remission_map);
		carmen_prob_models_free_compact_map(&local_compacted_variance_remission_map);

		carmen_prob_models_create_compact_map(&local_compacted_map, &local_map, -1.0);
		carmen_prob_models_create_compact_map(&local_compacted_mean_remission_map, &local_mean_remission_map, -1.0);
		carmen_prob_models_create_compact_map(&local_compacted_variance_remission_map, &local_variance_remission_map, -1.0);

		create_binary_map(&binary_map, local_compacted_mean_remission_map);

		carmen_prob_models_clear_carmen_map_using_compact_map(&local_map, &local_compacted_mean_remission_map, -1.0);
		carmen_prob_models_clear_carmen_map_using_compact_map(&local_mean_remission_map, &local_compacted_mean_remission_map, -1.0);
		carmen_prob_models_clear_carmen_map_using_compact_map(&local_variance_remission_map, &local_compacted_variance_remission_map, -1.0);
		carmen_prob_models_clear_carmen_map_using_compact_map(&local_sum_remission_map, &local_compacted_mean_remission_map, -1.0);
		carmen_prob_models_clear_carmen_map_using_compact_map(&local_sum_sqr_remission_map, &local_compacted_variance_remission_map, -1.0);
		carmen_prob_models_clear_carmen_map_using_compact_map(&local_count_remission_map, &local_compacted_mean_remission_map, -1.0);

		carmen_prob_models_clear_carmen_map_using_compact_map(&local_map, &local_compacted_map, -1.0);

		if (message_id > 1000000)
			message_id = 0;
	}
	message_id++;
	velodyne_data->last_timestamp = message->timestamp;

	if (message_id >= 0)
		return (1);
	else
		return (0);
}

//		carmen_FILE *fp;
//		fp = carmen_fopen("local.map", "w+");
//		carmen_map_write_all(fp, local_map.map,
//							local_map.config.x_size,
//							local_map.config.y_size,
//							local_map.config.resolution,
//							"",	"", "", "Generated by big_map",
//							NULL, 0, NULL, 0, NULL, 0);


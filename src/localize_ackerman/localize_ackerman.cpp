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
#include <carmen/robot_ackerman_messages.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/grid_mapping_interface.h>
#include <carmen/grid_mapping_messages.h>
#include <carmen/map_server_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/stereo_velodyne_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/grid_mapping_interface.h>


#include <prob_measurement_model.h>
#include <prob_map.h>

#include "localize_ackerman_core.h"
#include "localize_ackerman_messages.h"
#include "localize_ackerman_interface.h"
#include "localize_ackerman_velodyne.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "localize_ackerman_using_map.h"

double safe_range_above_sensors;

static int necessary_maps_available = 0;
static int use_raw_laser = 1;
static int use_velocity_prediction = 0;
static int mapping_mode = 0;

#define BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE 50
static carmen_base_ackerman_odometry_message base_ackerman_odometry_vector[BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE];
static int base_ackerman_odometry_index = -1;

#define FUSED_ODOMETRY_VECTOR_SIZE 50
static carmen_fused_odometry_message fused_odometry_vector[FUSED_ODOMETRY_VECTOR_SIZE];
static int fused_odometry_index = -1;

/* global variables */
carmen_map_t *new_map = NULL;
carmen_localize_ackerman_map_t map;
carmen_localize_ackerman_particle_filter_p filter;
carmen_localize_ackerman_summary_t summary;

carmen_map_t local_map;
carmen_map_t local_sum_remission_map;
carmen_map_t local_mean_remission_map;
carmen_map_t local_variance_remission_map;
carmen_map_t local_sum_sqr_remission_map;
carmen_map_t local_count_remission_map;

carmen_compact_map_t local_compacted_map;
carmen_compact_map_t local_compacted_mean_remission_map;
carmen_compact_map_t local_compacted_variance_remission_map;
carmen_localize_ackerman_binary_map_t binary_map;

carmen_robot_ackerman_laser_message front_laser;

carmen_pose_3D_t sensor_board_1_pose;
rotation_matrix *sensor_board_1_to_car_matrix;

int velodyne_viewer = 0;

double robot_wheel_radius;
carmen_robot_ackerman_config_t car_config;
double highest_sensor;

double x_origin = 0.0;
double y_origin = 0.0;

sensor_parameters_t *spherical_sensor_params;
sensor_data_t *spherical_sensor_data;

carmen_localize_ackerman_globalpos_message globalpos;

extern double laser_ranges[10000];
int number_of_sensors;
double max_range = 0.0;
carmen_pose_3D_t velodyne_pose;

cell_coords_t **map_cells_hit_by_each_rays = NULL;

carmen_point_t g_std;
int g_reinitiaze_particles = 10;
int number_of_threads = 10;

static int
get_fused_odometry_index_by_timestamp(double timestamp)
{
	double min_diff, diff;
	int min_index;

	min_diff = DBL_MAX;
	min_index = -1;

	for (int i = 0; i < FUSED_ODOMETRY_VECTOR_SIZE; i++)
	{
		diff = fabs(fused_odometry_vector[i].timestamp - timestamp);

		if (diff < min_diff)
		{
			min_diff = diff;
			min_index = i;
		}
	}

	return min_index;
}


static int
get_base_ackerman_odometry_index_by_timestamp(double timestamp)
{
	double min_diff, diff;
	int min_index;

	min_diff = DBL_MAX;
	min_index = -1;

	for (int i = 0; i < BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE; i++)
	{
		diff = fabs(base_ackerman_odometry_vector[i].timestamp - timestamp);

		if (diff < min_diff)
		{
			min_diff = diff;
			min_index = i;
		}
	}

	return min_index;
}


static void
equalize_image(IplImage *img)
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
debug_remission_map(carmen_velodyne_partial_scan_message *velodyne_message)
{

	int i, j;
	int  k, r, l;
	static IplImage *remission_map = NULL;
	static char *cell_touched = 0;

	double sin_theta = sin(summary.mean.theta);
	double cos_theta = cos(summary.mean.theta);

	carmen_vector_2D_t robot_position;
	robot_position.x = summary.mean.x - map.config.x_origin;
	robot_position.y = summary.mean.y - map.config.y_origin;

	if (remission_map == NULL)
	{
		remission_map = cvCreateImage(cvSize(2 * 360, spherical_sensor_params[0].vertical_resolution * 2), IPL_DEPTH_8U, 3);
		cell_touched = (char *)calloc(map.config.x_size * map.config.y_size, sizeof(char));
	}


	memset(cell_touched, 0, map.config.x_size * map.config.y_size * sizeof(char));
	for (l = 0, j = 0; j < spherical_sensor_params[0].vertical_resolution; j++, l+=2)
	{
		k = 0;
		r = spherical_sensor_params[0].ray_order[(spherical_sensor_params[0].vertical_resolution - 1) - j];
		for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
		{
			k = i;
			int index = ((int)((velodyne_message->partial_scan[i].angle + 180.0) / 0.5) % 720);
			if ((velodyne_message->partial_scan[k].distance[r]) != 0)
			{
				float range = (velodyne_message->partial_scan[k].distance[r] / 500.0);
				if (range < 50.0)
				{
					cell_coords_t map_cell = calc_global_cell_coordinate(&map_cells_hit_by_each_rays[k][r], &local_map.config, &robot_position, sin_theta, cos_theta);
					if (map_cell.x >= 0 && map_cell.y >= 0 && map_cell.x < map.config.x_size && map_cell.y < map.config.y_size)
					{
						if (/*!cell_touched[map_cell.x * map.config.x_size + map_cell.y]*/1)
						{
							cell_touched[map_cell.x * map.config.x_size + map_cell.y] = 1;
							remission_map->imageData[(l * remission_map->widthStep + 3 * index)] = 255 * (1.0 - map.carmen_mean_remission_map.map[map_cell.x][map_cell.y]);
							remission_map->imageData[(l * remission_map->widthStep + 3 * index) + 1] = 255 * (1.0 - map.carmen_mean_remission_map.map[map_cell.x][map_cell.y]);
							remission_map->imageData[(l * remission_map->widthStep + 3 * index) + 2] = 255 * (1.0 - map.carmen_mean_remission_map.map[map_cell.x][map_cell.y]);
						}
						else
							remission_map->imageData[(l * remission_map->widthStep + 3 * index)] = 255;
					}
					else
						remission_map->imageData[(l * remission_map->widthStep + 3 * index) + 2] = 255;
				}
				else
				{
					remission_map->imageData[(l * remission_map->widthStep + 3 * index)] = 255;
				}
			}
			else
			{
				remission_map->imageData[(l * remission_map->widthStep + 3 * index)] = 255;
			}

		}

	}

	for (l = 0, j = 0; j < spherical_sensor_params[0].vertical_resolution; j++, l+=2)
	{
		memcpy(&remission_map->imageData[(l + 1) * remission_map->widthStep], &remission_map->imageData[l * remission_map->widthStep], remission_map->widthStep);
	}

	equalize_image(remission_map);
	cvShowImage("remission_map", remission_map);
	cvWaitKey(33);

}



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void 
publish_globalpos(carmen_localize_ackerman_summary_p summary, double v, double phi, double timestamp)
{
	IPC_RETURN_TYPE err;

	if (g_reinitiaze_particles)
	{
		g_reinitiaze_particles--;
		return;
	}
		
	globalpos.timestamp = timestamp;
	globalpos.host = carmen_get_host();
	globalpos.globalpos = summary->mean;
	globalpos.globalpos_std = summary->std;
	globalpos.odometrypos = summary->odometry_pos;
	globalpos.globalpos_xy_cov = summary->xy_cov;
	globalpos.v = v;
	globalpos.phi = phi;
	globalpos.converged = summary->converged;

	if (fused_odometry_index == -1)
	{
		globalpos.pose.orientation.pitch = globalpos.pose.orientation.roll = globalpos.pose.position.z = 0.0;
		globalpos.velocity.x = globalpos.velocity.y = globalpos.velocity.z = 0.0;
	}
	else
	{	// Aproveita alguns dados da fused_odometry. 
		// Os valores referentes aa globalpos corrente sao escritos abaixo.
		globalpos.pose = fused_odometry_vector[get_fused_odometry_index_by_timestamp(timestamp)].pose; 	
		globalpos.velocity = fused_odometry_vector[get_fused_odometry_index_by_timestamp(timestamp)].velocity; 	
	}
	globalpos.pose.orientation.yaw = globalpos.globalpos.theta;

	globalpos.pose.position.x = globalpos.globalpos.x;
	globalpos.pose.position.y = globalpos.globalpos.y;
	globalpos.pose.position.z = 0;
	globalpos.velocity.x = v;
	
	//globalpos.pose.orientation.pitch = globalpos.pose.orientation.roll = 0.0;

	//static FILE *f = fopen("localize.txt", "w");
	//fprintf(f, "%f %f %f %f\n", globalpos.pose.position.x, globalpos.pose.position.y, globalpos.pose.orientation.yaw, timestamp);

	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &globalpos);
	carmen_test_ipc_exit(err, "Could not publish",
			CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
}


static void
publish_particles_name(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary,
			char *message_name, double timestamp)
{
	static carmen_localize_ackerman_particle_message pmsg;
	IPC_RETURN_TYPE err;

	pmsg.timestamp = timestamp;
	pmsg.host = carmen_get_host();
	pmsg.globalpos = summary->mean;
	pmsg.globalpos_std = summary->mean;
	pmsg.num_particles = filter->param->num_particles;
	pmsg.particles = (carmen_localize_ackerman_particle_ipc_p) filter->particles;

	err = IPC_publishData(message_name, &pmsg);
	carmen_test_ipc_exit(err, "Could not publish",
			CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME);
}


void 
publish_particles(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary, double timestamp)
{
	publish_particles_name(filter, summary, (char *) CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME, timestamp);
}


void 
publish_particles_prediction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary, double timestamp)
{
	publish_particles_name(filter, summary, (char *) CARMEN_LOCALIZE_ACKERMAN_PARTICLE_PREDICTION_NAME, timestamp);
}


void 
publish_particles_correction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary, double timestamp)
{
	publish_particles_name(filter, summary, (char *) CARMEN_LOCALIZE_ACKERMAN_PARTICLE_CORRECTION_NAME, timestamp);
}


void 
publish_sensor(carmen_localize_ackerman_particle_filter_p filter,
		carmen_localize_ackerman_summary_p summary,
		int num_readings,
		double *range,
		carmen_laser_laser_config_t laser_config,
		int front,
		double timestamp)
{
	static carmen_localize_ackerman_sensor_message sensor;
	IPC_RETURN_TYPE err;

	sensor.timestamp = timestamp;
	sensor.host = carmen_get_host();
	if(front) {
		sensor.pose.x = summary->mean.x + filter->param->front_laser_offset *
				cos(summary->mean.theta);
		sensor.pose.y = summary->mean.y + filter->param->front_laser_offset *
				sin(summary->mean.theta);
		sensor.pose.theta = summary->mean.theta;
		sensor.num_laser = 1;
	}
	else {
		sensor.pose.x = summary->mean.x + filter->param->rear_laser_offset *
				cos(summary->mean.theta + M_PI);
		sensor.pose.y = summary->mean.y + filter->param->rear_laser_offset *
				sin(summary->mean.theta + M_PI);
		sensor.pose.theta = summary->mean.theta + M_PI;
		sensor.num_laser = 2;
	}
	sensor.num_readings = num_readings;
	sensor.laser_skip = filter->param->laser_skip;
	sensor.config = laser_config;
	sensor.range = range;
	sensor.mask = filter->laser_mask;
	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME, &sensor);
	carmen_test_ipc_exit(err, "Could not publish",
			CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME);
}


void
publish_first_globalpos(carmen_localize_ackerman_initialize_message *initialize_msg)
{
	carmen_localize_ackerman_globalpos_message globalpos_ackerman_message;
	globalpos_ackerman_message.globalpos = *initialize_msg->mean;
	globalpos_ackerman_message.globalpos_std = *initialize_msg->std;
	globalpos_ackerman_message.odometrypos = *initialize_msg->std;

	globalpos_ackerman_message.timestamp = initialize_msg->timestamp;
	globalpos_ackerman_message.host = carmen_get_host();

	globalpos_ackerman_message.converged = 0;
	globalpos_ackerman_message.globalpos_xy_cov = 0.0;
	globalpos_ackerman_message.phi = 0.0;
	globalpos_ackerman_message.v = 0.0;
	
	carmen_localize_ackerman_publish_globalpos_message(&globalpos_ackerman_message);
}
///////////////////////////////////////////////////////////////////////////////////////////////


void
velodyne_variable_scan_localize(carmen_velodyne_variable_scan_message *message, int sensor)
{
	int odometry_index, fused_odometry_index;
	int velodyne_initilized;

	if (!necessary_maps_available || base_ackerman_odometry_index < 0)
		return;

	if (mapping_mode)
		return;

	odometry_index = get_base_ackerman_odometry_index_by_timestamp(message->timestamp);
	fused_odometry_index = get_fused_odometry_index_by_timestamp(message->timestamp);

	velodyne_initilized = localize_ackerman_velodyne_variable_scan(message, &spherical_sensor_params[sensor], &spherical_sensor_data[sensor], &(globalpos.velocity));
	if (!velodyne_initilized)
		return;

	carmen_localize_ackerman_run_with_velodyne_prediction(filter, &base_ackerman_odometry_vector[odometry_index],
			&fused_odometry_vector[fused_odometry_index], use_velocity_prediction,
								message->timestamp, car_config.distance_between_front_and_rear_axles);


	carmen_localize_ackerman_run_with_velodyne_correction(filter, &map, &local_compacted_map, &local_compacted_mean_remission_map, &local_compacted_variance_remission_map, &binary_map);


//	if (fabs(base_ackerman_odometry_vector[odometry_index].v) > 0.2)
	{
		carmen_localize_ackerman_run_with_velodyne_resample(filter);
	}

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize_velodyne(filter, &summary);

		publish_globalpos(&summary, base_ackerman_odometry_vector[odometry_index].v,
				base_ackerman_odometry_vector[odometry_index].phi, message->timestamp);
//		publish_particles_correction(filter, &summary, message->timestamp);
		publish_particles(filter, &summary, message->timestamp);
	}

}



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	int odometry_index, fused_odometry_index;
	int velodyne_initilized;

	if (!necessary_maps_available)
		return;

	if (mapping_mode)
		return;

	odometry_index = get_base_ackerman_odometry_index_by_timestamp(velodyne_message->timestamp);
	fused_odometry_index = get_fused_odometry_index_by_timestamp(velodyne_message->timestamp);

	velodyne_initilized = localize_ackerman_velodyne_partial_scan(velodyne_message, &spherical_sensor_params[0], &spherical_sensor_data[0], &(globalpos.velocity), fused_odometry_vector[fused_odometry_index].phi);
	if (!velodyne_initilized)
		return;


	carmen_localize_ackerman_run_with_velodyne_prediction(filter, &base_ackerman_odometry_vector[odometry_index],
				&fused_odometry_vector[fused_odometry_index], use_velocity_prediction,
									velodyne_message->timestamp, car_config.distance_between_front_and_rear_axles);

	carmen_localize_ackerman_run_with_velodyne_correction(filter, &map, &local_compacted_map, &local_compacted_mean_remission_map, &local_compacted_variance_remission_map, &binary_map);
	publish_particles(filter, &summary, velodyne_message->timestamp);
//	publish_particles_correction(filter, &summary, velodyne_message->timestamp);


//	if (fabs(base_ackerman_odometry_vector[odometry_index].v) > 0.2)
	{
		carmen_localize_ackerman_run_with_velodyne_resample(filter);
	}

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize_velodyne(filter, &summary);
		publish_globalpos(&summary, base_ackerman_odometry_vector[odometry_index].v,
				base_ackerman_odometry_vector[odometry_index].phi, velodyne_message->timestamp);
//		publish_particles_correction(filter, &summary, velodyne_message->timestamp);
//		publish_particles(filter, &summary, velodyne_message->timestamp);
	}

	if (g_reinitiaze_particles)
		carmen_localize_ackerman_initialize_particles_gaussians(filter, 1, &(summary.mean), &g_std);
}


static void
velodyne_variable_scan_message_handler1(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 1);
}


void
velodyne_variable_scan_message_handler2(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 2);
}


void
velodyne_variable_scan_message_handler3(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 3);
}


void
velodyne_variable_scan_message_handler4(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 4);
}


void
velodyne_variable_scan_message_handler5(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 5);
}


void
velodyne_variable_scan_message_handler6(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 6);
}


void
velodyne_variable_scan_message_handler7(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 7);
}


void
velodyne_variable_scan_message_handler8(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 8);
}


void
velodyne_variable_scan_message_handler9(carmen_velodyne_variable_scan_message *message)
{
	velodyne_variable_scan_localize(message, 9);
}


void
robot_ackerman_frontlaser_handler(carmen_robot_ackerman_laser_message *flaser)
{
	if (!necessary_maps_available)
		return;

	carmen_localize_ackerman_run(filter, &map, flaser, filter->param->front_laser_offset, 0, &base_ackerman_odometry_vector[base_ackerman_odometry_index], car_config.distance_between_front_and_rear_axles);

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize(filter, &summary, &map, flaser->num_readings,
				flaser->range, filter->param->front_laser_offset,
				flaser->config.angular_resolution,
				flaser->config.start_angle, 0);
		publish_globalpos(&summary, flaser->v, flaser->phi, flaser->timestamp);
		publish_particles(filter, &summary, flaser->timestamp);
		publish_sensor(filter, &summary, flaser->num_readings, flaser->range, flaser->config, 1, flaser->timestamp);
	}
}


static void
raw_laser_handler(carmen_laser_laser_message *laser)
{
	int odometry_index;
//	int i;

	if (!necessary_maps_available || base_ackerman_odometry_index < 0)
		return;

	odometry_index = get_base_ackerman_odometry_index_by_timestamp(laser->timestamp);

	carmen_localize_ackerman_run_with_raw_laser(filter, &map,
			laser, &base_ackerman_odometry_vector[odometry_index],
			filter->param->front_laser_offset, car_config.distance_between_front_and_rear_axles);

//	printf("FLASER %d", laser->num_readings);
//	for (i = 0; i < laser->num_readings; i++)
//	{
//		printf(" %lf", laser->range[i]);
//	}
//	printf(" %f %f %f 0 0 0\n", base_ackerman_odometry_vector[odometry_index].x, base_ackerman_odometry_vector[odometry_index].y, base_ackerman_odometry_vector[odometry_index].theta);

	if (filter->initialized)
	{
		carmen_localize_ackerman_summarize(filter, &summary, &map, laser->num_readings,
				laser->range, filter->param->front_laser_offset,
				laser->config.angular_resolution,
				laser->config.start_angle, 0);
		publish_globalpos(&summary, base_ackerman_odometry_vector[odometry_index].v,
				base_ackerman_odometry_vector[odometry_index].phi, laser->timestamp);
		publish_particles(filter, &summary, laser->timestamp);
		publish_sensor(filter, &summary, laser->num_readings, laser->range, laser->config, 1, laser->timestamp);
	}
}


static void
base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *msg)
{
	base_ackerman_odometry_index = (base_ackerman_odometry_index + 1) % BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE;
	base_ackerman_odometry_vector[base_ackerman_odometry_index] = *msg;

	localalize_using_map_set_robot_pose_into_the_map(msg->v, msg->phi, msg->timestamp);
}


static void
fused_odometry_handler(carmen_fused_odometry_message *msg)
{
	IPC_RETURN_TYPE err;

	static int is_first_fused_odometry_message = 1;

	if (is_first_fused_odometry_message)
	{
		is_first_fused_odometry_message = 0;
		return;
	}


	fused_odometry_index = (fused_odometry_index + 1) % FUSED_ODOMETRY_VECTOR_SIZE;
	fused_odometry_vector[fused_odometry_index] = *msg;

	if (mapping_mode)
	{

		globalpos.timestamp = msg->timestamp;
		globalpos.host = carmen_get_host();
		globalpos.v = msg->velocity.x;
		globalpos.phi = msg->phi;
		globalpos.pose = msg->pose;
		globalpos.velocity = msg->velocity;
		globalpos.globalpos.x = globalpos.pose.position.x;
		globalpos.globalpos.y = globalpos.pose.position.y;
		globalpos.globalpos.theta = globalpos.pose.orientation.yaw;

		err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &globalpos);
		carmen_test_ipc_exit(err, "Could not publish",
				CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

		return;
	}

}


void
carmen_localize_ackerman_initialize_handler(carmen_localize_ackerman_initialize_message *initialize_msg)
{
//	static int first = 1;

	if (initialize_msg->distribution == CARMEN_INITIALIZE_GAUSSIAN)
	{
		carmen_localize_ackerman_initialize_particles_gaussians(filter,
			initialize_msg->num_modes, initialize_msg->mean, initialize_msg->std);

		g_std = initialize_msg->std[0];
		g_reinitiaze_particles = 10;

//		if (first) // Alberto: isto estava impedindo de se poder reinicializar o log em outro ponto
//		{
			filter->last_timestamp = initialize_msg->timestamp;
//			first = 0;
//		}

		publish_first_globalpos(initialize_msg); // Alberto: se publicar pode sujar o mapa devido a inicializacao.
	}
	else if (initialize_msg->distribution == CARMEN_INITIALIZE_UNIFORM)
	{
		//todo pode dar problema aqui se o mapa nao estiver inicializado
		carmen_localize_ackerman_initialize_particles_uniform(filter, &front_laser, &map);
		publish_particles(filter, &summary, initialize_msg->timestamp);
	}
	necessary_maps_available = 0;

}


static void
localize_map_update_handler(carmen_map_server_localize_map_message *message)
{
	carmen_map_server_localize_map_message_to_localize_map(message, &map);

	x_origin = message->config.x_origin;
	y_origin = message->config.y_origin;

	necessary_maps_available = 1;
}


static void
globalpos_query_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, 
		void *clientData __attribute__ ((unused)))
{
	IPC_RETURN_TYPE err;
	carmen_localize_ackerman_globalpos_message globalpos;
	//  FORMATTER_PTR formatter;

	if (!necessary_maps_available)
		return;

	/* formatter = */IPC_msgInstanceFormatter(msgRef);
	IPC_freeByteArray(callData);

	globalpos.timestamp = carmen_get_time();
	globalpos.host = carmen_get_host();
	globalpos.globalpos = summary.mean;
	globalpos.globalpos_std = summary.std;
	globalpos.globalpos_xy_cov = summary.xy_cov;
	globalpos.odometrypos = summary.odometry_pos;
	globalpos.converged = summary.converged;

	err = IPC_respondData(msgRef, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &globalpos);
	carmen_test_ipc(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
}


static void 
map_query_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;
	carmen_localize_ackerman_map_query_message msg;
	carmen_localize_ackerman_map_message response;

#ifndef NO_ZLIB
	unsigned long compress_buf_size;
	int compress_return;
	unsigned char *compressed_map;
#endif

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &msg,
			sizeof(carmen_localize_ackerman_map_query_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall",
			IPC_msgInstanceName(msgRef));

	response.config = map.config;

	if (msg.map_is_global_likelihood) {
		response.map_is_global_likelihood = 1;
		response.data = (unsigned char *)map.complete_gprob;
		response.size = map.config.x_size*map.config.y_size*sizeof(float);
	} else {
		response.map_is_global_likelihood = 0;
		response.data = (unsigned char *)map.complete_prob;
		response.size = map.config.x_size*map.config.y_size*sizeof(float);
	}

#ifndef NO_ZLIB
	compress_buf_size = response.size*1.01+12;
	compressed_map = (unsigned char *)calloc(compress_buf_size, sizeof(unsigned char));
	carmen_test_alloc(compressed_map);
	compress_return = carmen_compress(
			(unsigned char *)compressed_map,
			(unsigned long *)&compress_buf_size,
			(unsigned char *)response.data,
			(unsigned long)response.size,
			Z_DEFAULT_COMPRESSION);
	if (compress_return != Z_OK) {
		free(compressed_map);
		response.compressed = 0;
	} else {
		response.size = compress_buf_size;
		response.data = compressed_map;
		response.compressed = 1;
	}
#else
	response.compressed = 0;
#endif

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	err = IPC_respondData(msgRef, CARMEN_LOCALIZE_ACKERMAN_MAP_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_LOCALIZE_ACKERMAN_MAP_NAME);
}


static void
shutdown_localize(int x)
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
	int i;

	spherical_sensor_params = (sensor_parameters_t *)calloc(number_of_sensors, sizeof(sensor_parameters_t));
	carmen_test_alloc(spherical_sensor_params);

	spherical_sensor_data = (sensor_data_t *)calloc(number_of_sensors, sizeof(sensor_data_t));
	carmen_test_alloc(spherical_sensor_data);

	carmen_param_t param_list[] =
	{
			{(char*)"localize_ackerman", (char*)"velodyne", CARMEN_PARAM_ONOFF, &spherical_sensor_params[0].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne1", CARMEN_PARAM_ONOFF, &spherical_sensor_params[1].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne2", CARMEN_PARAM_ONOFF, &spherical_sensor_params[2].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne3", CARMEN_PARAM_ONOFF, &spherical_sensor_params[3].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne4", CARMEN_PARAM_ONOFF, &spherical_sensor_params[4].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne5", CARMEN_PARAM_ONOFF, &spherical_sensor_params[5].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne6", CARMEN_PARAM_ONOFF, &spherical_sensor_params[6].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne7", CARMEN_PARAM_ONOFF, &spherical_sensor_params[7].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne8", CARMEN_PARAM_ONOFF, &spherical_sensor_params[8].alive, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne9", CARMEN_PARAM_ONOFF, &spherical_sensor_params[9].alive, 0, NULL},

			{(char*)"localize_ackerman", (char*)"velodyne_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne1_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[1].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne2_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne3_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne4_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne5_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne6_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne7_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne8_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].log_odds.log_odds_occ, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne9_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].log_odds.log_odds_occ, 0, NULL},


			{(char*)"localize_ackerman", (char*)"velodyne_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne1_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[1].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne2_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne3_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne4_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne5_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne6_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne7_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne8_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].log_odds.log_odds_free, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne9_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].log_odds.log_odds_free, 0, NULL},

			{(char*)"localize_ackerman", (char*)"velodyne_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne1_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[1].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne2_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne3_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne4_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne5_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne6_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne7_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne8_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].log_odds.log_odds_l0, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne9_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].log_odds.log_odds_l0, 0, NULL},

			{(char*)"localize_ackerman", (char*)"velodyne_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne1_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[1].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne2_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne3_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne4_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne5_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne6_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne7_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne8_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].unexpeted_delta_range_sigma, 0, NULL},
			{(char*)"localize_ackerman", (char*)"stereo_velodyne9_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].unexpeted_delta_range_sigma, 0, NULL},


			{(char*)"localize_ackerman", (char*)"unsafe_height_above_ground", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].unsafe_height_above_ground, 0, NULL},


			{(char*)"localize_ackerman",  (char*)"velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].range_max_factor, 0, NULL}


	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	for (i = 0; i < number_of_sensors; i++)
	{
		if (i == STEREO_MAPPING_SENSOR_INDEX)
			continue;

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
		spherical_sensor_params[i].sensor_to_board_matrix = NULL;
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
			spherical_sensor_data[i].processed_intensity[i] = NULL;
			spherical_sensor_data[i].ray_hit_the_robot[j] = NULL;
		}

		if (spherical_sensor_params[i].alive)
		{
			spherical_sensor_params[i].name = (char *)calloc(strlen(param_list[i].variable) + 1, sizeof(char));
			strcpy(spherical_sensor_params[i].name, param_list[i].variable);
		}
	}
}


static int *
generates_ray_order(int size)
{
	int i;

	int *ray_order = (int *)malloc(size * sizeof(int));
	carmen_test_alloc(ray_order);

	for (i = 0; i < size; i++)
		ray_order[i] = i;

	return ray_order;
}


static void
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
				{spherical_sensor_params[0].name, (char*)"time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].time_spent_by_each_scan, 0, NULL},

		};

		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		init_velodyne_points(&spherical_sensor_data[0].points, &spherical_sensor_data[0].intensity, &spherical_sensor_data[0].robot_pose,
				&spherical_sensor_data[0].robot_velocity, &spherical_sensor_data[0].robot_timestamp, &spherical_sensor_data[0].robot_phi);
		spherical_sensor_params[0].sensor_to_board_matrix = create_rotation_matrix(spherical_sensor_params[0].pose.orientation);
		spherical_sensor_data[0].point_cloud_index = 0;
		carmen_prob_models_alloc_sensor_data(&spherical_sensor_data[0], spherical_sensor_params[0].vertical_resolution, number_of_threads);

//		spherical_sensor_params[0].remission_calibration = (double *) calloc(256 * spherical_sensor_params[0].vertical_resolution, sizeof(double));
//		FILE *f = fopen("../data/remission_calibration.txt", "r");
//		for (i = 0; i < 256 * spherical_sensor_params[0].vertical_resolution; i++)
//		{
//			fscanf(f, "%lf", &spherical_sensor_params[0].remission_calibration[i]);
//		}
//		fclose(f);

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
			init_velodyne_points(&spherical_sensor_data[i].points, &spherical_sensor_data[i].intensity, &spherical_sensor_data[i].robot_pose,
					&spherical_sensor_data[i].robot_velocity, &spherical_sensor_data[i].robot_timestamp, &spherical_sensor_data[i].robot_phi);
			spherical_sensor_params[i].sensor_to_board_matrix = create_rotation_matrix(spherical_sensor_params[i].pose.orientation);
			spherical_sensor_data[i].point_cloud_index = 0;
			carmen_prob_models_alloc_sensor_data(&spherical_sensor_data[i], spherical_sensor_params[i].vertical_resolution, number_of_threads);

			//TODO : tem que fazer esta medida para as cameras igual foi feito para o velodyne
			spherical_sensor_params[i].delta_difference_mean = (double *)calloc(50, sizeof(double));
			spherical_sensor_params[i].delta_difference_stddev = (double *)calloc(50, sizeof(double));
			for (j = 0; j < 50; j++)
				spherical_sensor_params[i].delta_difference_stddev[j] = 1.0;

		}
	}
}


static void 
read_parameters(int argc, char **argv, carmen_localize_ackerman_param_p param, ProbabilisticMapParams *p_map_params)
{
	double integrate_angle_deg;

	integrate_angle_deg = 1.0;

	carmen_param_t param_list[] = 
	{
			{(char *) "localize_ackerman", 	(char *) "velocity_noise_velocity", CARMEN_PARAM_DOUBLE, &param->velocity_noise_velocity, 0, NULL},
			{(char *) "localize_ackerman", 	(char *) "velocity_noise_phi", CARMEN_PARAM_DOUBLE, &param->velocity_noise_phi, 0, NULL},
			{(char *) "localize_ackerman", 	(char *) "phi_noise_phi", CARMEN_PARAM_DOUBLE, &param->phi_noise_phi, 0, NULL},
			{(char *) "localize_ackerman", 	(char *) "phi_noise_velocity", CARMEN_PARAM_DOUBLE, &param->phi_noise_velocity, 0, NULL},
			{(char *) "localize_ackerman", 	(char *) "use_velocity_prediction", CARMEN_PARAM_ONOFF, &param->use_velocity_prediction, 0, NULL},

			{(char *)"robot", (char*)"frontlaser_offset", CARMEN_PARAM_DOUBLE, &param->front_laser_offset, 0, NULL},
			{(char *)"robot", (char*)"rearlaser_offset", CARMEN_PARAM_DOUBLE, &param->rear_laser_offset, 0, NULL},
			{(char *)"robot", (char*)"length", CARMEN_PARAM_DOUBLE, &car_config.length, 0, NULL},
			{(char *)"robot", (char*)"width", CARMEN_PARAM_DOUBLE, &car_config.width, 0, NULL},
			{(char *)"robot", (char*)"distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &car_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
			{(char *)"robot",  (char*)"distance_between_front_and_rear_axles",		CARMEN_PARAM_DOUBLE, &car_config.distance_between_front_and_rear_axles, 1, NULL},

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

			{(char *)"velodyne", (char*)"x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
			{(char *)"velodyne", (char*)"y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
			{(char *)"velodyne", (char*)"z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
			{(char *)"velodyne", (char*)"roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
			{(char *)"velodyne", (char*)"pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
			{(char *)"velodyne", (char*)"yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

			{(char *)"localize_ackerman",  (char*)"number_of_sensors", CARMEN_PARAM_INT, &number_of_sensors, 0, NULL},
			{(char *)"localize_ackerman", (char*)"safe_range_above_sensors", CARMEN_PARAM_DOUBLE, &safe_range_above_sensors, 0, NULL},
			{(char *)"localize_ackerman", (char*)"correction_type", CARMEN_PARAM_INT, &param->correction_type, 0, NULL},

			{(char *)"localize_ackerman", (char*)"swarm_max_particle_velocity", CARMEN_PARAM_DOUBLE, &param->swarm_max_particle_velocity, 0, NULL},
			{(char *)"localize_ackerman", (char*)"swarm_max_particle_angular_velocity", CARMEN_PARAM_DOUBLE, &param->swarm_max_particle_angular_velocity, 0, NULL},
			{(char *)"localize_ackerman", (char*)"max_particle_displacement", CARMEN_PARAM_DOUBLE, &param->max_particle_displacement, 0, NULL},
			{(char *)"localize_ackerman", (char*)"max_particle_angular_displacement", CARMEN_PARAM_DOUBLE, &param->max_particle_angular_displacement, 0, NULL},
			{(char *)"localize_ackerman", (char*)"de_crossover_rate", CARMEN_PARAM_DOUBLE, &param->de_crossover_rate, 0, NULL},
			{(char *)"localize_ackerman", (char*)"de_mutation_rate", CARMEN_PARAM_DOUBLE, &param->de_mutation_rate, 0, NULL},
			{(char *)"localize_ackerman", (char*)"de_num_iteration", CARMEN_PARAM_INT, &param->de_num_iteration, 0, NULL},
			{(char *)"localize_ackerman", (char*)"swarm_num_iteration", CARMEN_PARAM_INT, &param->swarm_num_iteration, 0, NULL},
			{(char *)"localize_ackerman", (char*)"jump_size", CARMEN_PARAM_INT, &param->jump_size, 0, NULL},

			{(char *)"mapper", (char*)"map_log_odds_max", CARMEN_PARAM_INT, &p_map_params->log_odds_max, 0, NULL},
			{(char *)"mapper", (char*)"map_log_odds_min", CARMEN_PARAM_INT, &p_map_params->log_odds_min, 0, NULL},
			{(char *)"mapper", (char*)"map_log_odds_bias", CARMEN_PARAM_INT, &p_map_params->log_odds_bias, 0, NULL},
			{(char *)"mapper", (char*)"map_grid_res", CARMEN_PARAM_DOUBLE, &p_map_params->grid_res, 0, NULL},
			{(char *)"mapper", (char*)"map_range_factor", CARMEN_PARAM_DOUBLE, &p_map_params->range_factor, 0, NULL},
	};

	//p_map_params->grid_res = 0.1;

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

	carmen_map_config_t main_map_config;

	main_map_config.resolution = p_map_params->grid_res;
	main_map_config.x_size = 60.0 / p_map_params->grid_res;
	main_map_config.y_size = 60.0 / p_map_params->grid_res;

	localalize_using_map_initialize(&main_map_config);

	localize_ackerman_velodyne_laser_read_parameters(argc, argv);

	param->xy_uncertainty_due_to_grid_resolution = (p_map_params->grid_res / 2.0) * (p_map_params->grid_res / 2.0);
	param->yaw_uncertainty_due_to_grid_resolution = asin((p_map_params->grid_res / 2.0) / max_range) * asin((p_map_params->grid_res / 2.0) / max_range);
	
	carmen_param_allow_unfound_variables(1);

	carmen_param_t param_optional_list[] = 
	{
			{(char *)"localize_ackerman", (char*)"use_raw_laser", CARMEN_PARAM_ONOFF, &use_raw_laser, 0, NULL},
			{(char *)"commandline", (char*)"mapping_mode", CARMEN_PARAM_ONOFF, &mapping_mode, 0, NULL},
			{(char *)"commandline", (char*)"velodyne_viewer", CARMEN_PARAM_ONOFF, &velodyne_viewer, 0, NULL}
	};

	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));
}


int 
register_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	/* register globalpos message */
	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

	/* register robot particle message */
	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_PREDICTION_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_CORRECTION_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME);

	/* register sensor message */
	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_SENSOR_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME);

	/* register initialize message */
	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME);

	/* register map request message */
	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_MAP_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_MAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_MAP_NAME);

	/* register globalpos request message */
	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME, IPC_VARIABLE_LENGTH, CARMEN_DEFAULT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME);

	carmen_velodyne_define_messages();

	return 0;
}


static void
subscribe_to_ipc_message()
{
	IPC_RETURN_TYPE err;

	/* subscribe to initialization messages */
	carmen_localize_ackerman_subscribe_initialize_message(NULL, (carmen_handler_t) carmen_localize_ackerman_initialize_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) fused_odometry_handler,	CARMEN_SUBSCRIBE_LATEST);

	/* Subscribe to front and rear laser messages */
	if (!mapping_mode)
	{
		if (use_raw_laser)
		{
			carmen_laser_subscribe_laser1_message(NULL, (carmen_handler_t) raw_laser_handler, CARMEN_SUBSCRIBE_LATEST);
			carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_handler, CARMEN_SUBSCRIBE_LATEST);
		}
		else
		{
			carmen_robot_ackerman_subscribe_frontlaser_message(&front_laser, (carmen_handler_t) robot_ackerman_frontlaser_handler, CARMEN_SUBSCRIBE_LATEST);
		}


		carmen_map_server_subscribe_localize_map_message(NULL,
				(carmen_handler_t) localize_map_update_handler, CARMEN_SUBSCRIBE_LATEST);

		/* subscribe to map request message */
		err = IPC_subscribe(CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME, map_query_handler, NULL);
		carmen_test_ipc(err, "Could not subscribe", CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME);
		IPC_setMsgQueueLength(CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME, 1);

		/* subscribe to globalpos request message */
		err = IPC_subscribe(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME, globalpos_query_handler, NULL);
		carmen_test_ipc(err, "Could not subscribe", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME);
		IPC_setMsgQueueLength(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME, 1);

		if (spherical_sensor_params[0].alive)
		{
			carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);
		}
		if (spherical_sensor_params[1].alive)
		{
			carmen_stereo_velodyne_subscribe_scan_message(1, NULL,
					(carmen_handler_t)velodyne_variable_scan_message_handler1,
					CARMEN_SUBSCRIBE_LATEST);
		}

		if (spherical_sensor_params[2].alive)
		{
			carmen_stereo_velodyne_subscribe_scan_message(2, NULL,
					(carmen_handler_t)velodyne_variable_scan_message_handler2,
					CARMEN_SUBSCRIBE_LATEST);
		}

		if (spherical_sensor_params[3].alive)
		{
			carmen_stereo_velodyne_subscribe_scan_message(3, NULL,
					(carmen_handler_t)velodyne_variable_scan_message_handler3,
					CARMEN_SUBSCRIBE_LATEST);
		}

		if (spherical_sensor_params[4].alive)
		{
			carmen_stereo_velodyne_subscribe_scan_message(4, NULL,
					(carmen_handler_t)velodyne_variable_scan_message_handler4,
					CARMEN_SUBSCRIBE_LATEST);
		}

		if (spherical_sensor_params[5].alive)
		{
			carmen_stereo_velodyne_subscribe_scan_message(5, NULL,
					(carmen_handler_t)velodyne_variable_scan_message_handler5,
					CARMEN_SUBSCRIBE_LATEST);
		}

		if (spherical_sensor_params[6].alive)
		{
			carmen_stereo_velodyne_subscribe_scan_message(6, NULL,
					(carmen_handler_t)velodyne_variable_scan_message_handler6,
					CARMEN_SUBSCRIBE_LATEST);
		}

		if (spherical_sensor_params[7].alive)
		{
			carmen_stereo_velodyne_subscribe_scan_message(7, NULL,
					(carmen_handler_t)velodyne_variable_scan_message_handler7,
					CARMEN_SUBSCRIBE_LATEST);
		}

		if (spherical_sensor_params[8].alive)
		{
			carmen_stereo_velodyne_subscribe_scan_message(8, NULL,
					(carmen_handler_t)velodyne_variable_scan_message_handler8,
					CARMEN_SUBSCRIBE_LATEST);
		}

		if (spherical_sensor_params[9].alive)
		{
			carmen_stereo_velodyne_subscribe_scan_message(9, NULL,
					(carmen_handler_t)velodyne_variable_scan_message_handler9,
					CARMEN_SUBSCRIBE_LATEST);
		}
	}
}


static void
init_local_maps(ProbabilisticMapParams map_params)
{
	init_probabilistic_grid_map_model(&map_params, NULL);

	init_carmen_map(&map_params, &local_map);
	init_carmen_map(&map_params, &local_mean_remission_map);
	init_carmen_map(&map_params, &local_sum_remission_map);
	init_carmen_map(&map_params, &local_sum_sqr_remission_map);
	init_carmen_map(&map_params, &local_count_remission_map);
	init_carmen_map(&map_params, &local_variance_remission_map);

	local_compacted_map.coord_x = NULL;
	local_compacted_map.coord_y = NULL;
	local_compacted_map.value = NULL;
	local_compacted_map.config.map_name = NULL;
	local_compacted_map.number_of_known_points_on_the_map = 0;

	local_compacted_mean_remission_map.coord_x = NULL;
	local_compacted_mean_remission_map.coord_y = NULL;
	local_compacted_mean_remission_map.value = NULL;
	local_compacted_mean_remission_map.config.map_name = NULL;
	local_compacted_mean_remission_map.number_of_known_points_on_the_map = 0;

	local_compacted_variance_remission_map.coord_x = NULL;
	local_compacted_variance_remission_map.coord_y = NULL;
	local_compacted_variance_remission_map.value = NULL;
	local_compacted_variance_remission_map.config.map_name = NULL;
	local_compacted_variance_remission_map.number_of_known_points_on_the_map = 0;

	memset(&binary_map, 0, sizeof(carmen_localize_ackerman_binary_map_t));
}


static void
init_localize_map()
{
	map.carmen_map.complete_map = NULL;
	map.complete_distance = NULL;
	map.complete_gprob = NULL;
	map.complete_prob = NULL;
	map.complete_x_offset = NULL;
	map.complete_y_offset = NULL;

	map.carmen_map.map = NULL;
	map.distance = NULL;
	map.gprob = NULL;
	map.prob = NULL;
	map.x_offset = NULL;
	map.y_offset = NULL;
}


int 
main(int argc, char **argv) 
{ 
	carmen_localize_ackerman_param_t param;
	ProbabilisticMapParams map_params;

	/* initialize carmen */
	carmen_randomize(&argc, &argv);
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	/* Setup exit handler */
	signal(SIGINT, shutdown_localize);

	/* Initialize all the relevant parameters */
	read_parameters(argc, argv, &param, &map_params);

#ifndef OLD_MOTION_MODEL
	param.motion_model = carmen_localize_ackerman_motion_initialize(argc, argv);
#endif

	/* Allocate memory for the particle filter */
	filter = carmen_localize_ackerman_particle_filter_new(&param);

	init_localize_map();
	init_local_maps(map_params);

	/* register localize related messages */
	register_ipc_messages();

	/* subscribe to localize related messages */
	subscribe_to_ipc_message();

	/* Loop forever */
	carmen_ipc_dispatch();

	return 0;
}

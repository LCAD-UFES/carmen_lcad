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
#include <prob_map.h>
#include "localize_ackerman_core.h"
#include "localize_ackerman_motion.h"
#include "localize_ackerman_velodyne.h"
#include <prob_measurement_model.h>
#include <omp.h>
#include <carmen/rotation_geometry.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/task_manager_interface.h>

/* gains for gradient descent */

#define K_T   0.0001
#define K_ROT 0.00001

int number_of_sensors;
sensor_parameters_t *spherical_sensor_params;
sensor_data_t *spherical_sensor_data;
carmen_lidar_config lidar_config[MAX_NUMBER_OF_LIDARS];
carmen_pose_3D_t sensor_board_1_pose;
double robot_wheel_radius;
double highest_sensor;
char *calibration_file = NULL;
int number_of_threads = 1;
carmen_pose_3D_t velodyne_pose;
rotation_matrix *sensor_board_1_to_car_matrix;
carmen_semi_trailer_config_t semi_trailer_config;
carmen_robot_ackerman_config_t 	car_config;
int robot_publish_odometry;
double safe_range_above_sensors;
int use_raw_laser = 1;
int mapping_mode = 0;
int velodyne_viewer = 0;
char *save_globalpos_file = NULL;
double save_globalpos_timestamp = 0.0;


void
carmen_localize_ackerman_incorporate_IMU(carmen_localize_ackerman_particle_filter_p filter,
		carmen_xsens_global_quat_message *xsens_global_quat_message,
		double distance_between_front_and_rear_axles, double dt)
{
	double v_step, phi_step;
	carmen_pose_3D_t robot_pose;

	if (fabs(dt) > 3.0) // Possivelmente reposicionamento do robo na interface
		return;

	if (!xsens_global_quat_message)
		return;

	double average_v = filter->particles[0].v;

	filter->distance_travelled += fabs(average_v * dt);

	for (int i = 0; i < filter->param->num_particles; i++)
	{
		filter->particles[i].v += 1.1 * (xsens_global_quat_message->m_acc.x + 0.2) * dt + carmen_gaussian_random(0.0, 0.01);
		double v = filter->particles[i].v;
		if (fabs(average_v) > 0.2)
			filter->particles[i].phi = atan2((xsens_global_quat_message->m_gyr.z + carmen_gaussian_random(0.0, 0.01)) * distance_between_front_and_rear_axles, v);
		else
			filter->particles[i].phi = 0.0;
		double phi = filter->particles[i].phi;

		if (i != 0)
		{
			robot_pose.position.x = filter->particles[i].x;
			robot_pose.position.y = filter->particles[i].y;
			robot_pose.orientation.yaw = filter->particles[i].theta;

			v_step = v + carmen_gaussian_random(0.0,
					fabs(filter->param->velocity_noise_velocity * v) +
					fabs(filter->param->velocity_noise_phi * phi));

			if (fabs(v) > 0.05)
			{
				filter->particles[i].phi_bias += carmen_gaussian_random(0.0, filter->param->phi_bias_std);
				filter->particles[i].phi_bias = carmen_clamp(-0.0175 / 2.0, filter->particles[i].phi_bias, 0.0175 / 2.0);
			}
			phi_step = phi + filter->particles[i].phi_bias + carmen_gaussian_random(0.0,
					fabs(filter->param->phi_noise_phi * phi) +
					fabs(filter->param->phi_noise_velocity * v));
			phi_step = carmen_clamp(-M_PI/4.0, phi_step, M_PI/4.0);

			robot_pose = carmen_ackerman_interpolated_robot_position_at_time(robot_pose, dt, v_step, phi_step, distance_between_front_and_rear_axles);

			filter->particles[i].x = robot_pose.position.x + carmen_gaussian_random(0.0, filter->param->xy_uncertainty_due_to_grid_resolution);
			filter->particles[i].y = robot_pose.position.y + carmen_gaussian_random(0.0, filter->param->xy_uncertainty_due_to_grid_resolution);
			filter->particles[i].theta = carmen_normalize_theta(
					robot_pose.orientation.yaw + carmen_gaussian_random(0.0, filter->param->yaw_uncertainty_due_to_grid_resolution));
		}
		else
		{	// Keep the mean particle of the previous run intact
			robot_pose.position.x = filter->particles[i].x;
			robot_pose.position.y = filter->particles[i].y;
			robot_pose.orientation.yaw = filter->particles[i].theta;

			v_step = v;
			phi_step = phi + filter->particles[i].phi_bias;

			robot_pose = carmen_ackerman_interpolated_robot_position_at_time(robot_pose, dt, v_step, phi_step, distance_between_front_and_rear_axles);

			filter->particles[i].x = robot_pose.position.x;
			filter->particles[i].y = robot_pose.position.y;
			filter->particles[i].theta = carmen_normalize_theta(robot_pose.orientation.yaw);
		}
	}
}


void
carmen_localize_ackerman_incorporate_velocity_odometry(carmen_localize_ackerman_particle_filter_p filter,
		double v, double phi, double distance_between_front_and_rear_axles, double dt)
{
	double v_step, phi_step;
	carmen_pose_3D_t robot_pose;

	if (fabs(dt) > 3.0) // Possivelmente reposicionamento do robo na interface
		return;

//	FILE *caco = fopen("caco_gpos.txt", "a");
//	fprintf(caco, "%lf ", dt);
//	fflush(caco);
//	fclose(caco);

	filter->distance_travelled += fabs(v * dt);

	for (int i = 0; i < filter->param->num_particles; i++)
	{
		if (i != 0)
		{
			robot_pose.position.x = filter->particles[i].x;
			robot_pose.position.y = filter->particles[i].y;
			robot_pose.orientation.yaw = filter->particles[i].theta;

			v_step = v + carmen_gaussian_random(0.0,
					fabs(filter->param->velocity_noise_velocity * v) +
					fabs(filter->param->velocity_noise_phi * phi) + filter->param->v_uncertainty_at_zero_v);

			if (fabs(v) > 0.05)
			{
				// This update is similar to one used in "Map-Based Precision Vehicle Localization in Urban Environments" for updating the gps bias. 
				// In this paper, however, an additional factor is used for slowly pulling the bias towards zero. 
				// Using the rule from the paper, the update would be: phi = phi * gamma + N(0, phi_bias_std), where gamma = 0.9999.
				filter->particles[i].phi_bias += carmen_gaussian_random(0.0, filter->param->phi_bias_std);

				// 0.0175 radians is approx 1 degree
				filter->particles[i].phi_bias = carmen_clamp(-0.0175 / 2.0, filter->particles[i].phi_bias, 0.0175 / 2.0);
			}
			phi_step = phi + filter->particles[i].phi_bias + carmen_gaussian_random(0.0,
					fabs(filter->param->phi_noise_phi * phi) +
					fabs(filter->param->phi_noise_velocity * v));
			phi_step = carmen_clamp(-M_PI/4.0, phi_step, M_PI/4.0);

			robot_pose = carmen_ackerman_interpolated_robot_position_at_time(robot_pose, dt, v_step, phi_step, distance_between_front_and_rear_axles);

			filter->particles[i].x = robot_pose.position.x + carmen_gaussian_random(0.0, filter->param->xy_uncertainty_due_to_grid_resolution);
			filter->particles[i].y = robot_pose.position.y + carmen_gaussian_random(0.0, filter->param->xy_uncertainty_due_to_grid_resolution);
			filter->particles[i].theta = carmen_normalize_theta(
					robot_pose.orientation.yaw + carmen_gaussian_random(0.0, filter->param->yaw_uncertainty_due_to_grid_resolution));
			filter->particles[i].phi = phi_step;
			filter->particles[i].v = v_step;
		}
		else
		{
			// Keep the mean particle of the previous run intact
			// Note: This kind of elitism may introduce a bias in the filter.
			robot_pose.position.x = filter->particles[i].x;
			robot_pose.position.y = filter->particles[i].y;
			robot_pose.orientation.yaw = filter->particles[i].theta;

			v_step = v;
			phi_step = phi + filter->particles[i].phi_bias;

			robot_pose = carmen_ackerman_interpolated_robot_position_at_time(robot_pose, dt, v_step, phi_step, distance_between_front_and_rear_axles);

			filter->particles[i].x = robot_pose.position.x;
			filter->particles[i].y = robot_pose.position.y;
			filter->particles[i].theta = carmen_normalize_theta(robot_pose.orientation.yaw);
			filter->particles[i].phi = phi_step;
			filter->particles[i].v = v_step;
		}
	}
}

/* incorporate a single odometry reading into the particle filter */

void
carmen_localize_ackerman_incorporate_odometry(carmen_localize_ackerman_particle_filter_p filter,
		double v, double phi, double L, double dt)
{
	int i, backwards;
	double delta_t, delta_theta;
	double downrange, crossrange, turn;

	if (fabs(dt) > 3.0) // Possivelmente reposicionamento do robo na interface
		return;
		
	delta_t = v * dt;
	delta_theta = (delta_t / L) * tan(phi);

	backwards = delta_t < 0;

	delta_t = delta_t < 0 ? -delta_t : delta_t;

	filter->distance_travelled += delta_t;

	for (i = 0; i < filter->param->num_particles; i++)
	{
		downrange = carmen_localize_ackerman_sample_noisy_downrange(delta_t, delta_theta, filter->param->motion_model);
		crossrange = carmen_localize_ackerman_sample_noisy_crossrange(delta_t, delta_theta, filter->param->motion_model);
		turn = carmen_localize_ackerman_sample_noisy_turn(delta_t, delta_theta, filter->param->motion_model);

		if (backwards)
		{
			filter->particles[i].x -= downrange * cos(filter->particles[i].theta + turn/4.0) +
						  crossrange * cos(filter->particles[i].theta + turn/2.0 + M_PI/2.0);
			filter->particles[i].y -= downrange * sin(filter->particles[i].theta + turn/2.0) +
						  crossrange * sin(filter->particles[i].theta + turn/2.0 + M_PI/2.0);
		}
		else
		{
			filter->particles[i].x += downrange * cos(filter->particles[i].theta + turn/2.0) +
						  crossrange * cos(filter->particles[i].theta + turn/2.0 + M_PI/2.0);
			filter->particles[i].y += downrange * sin(filter->particles[i].theta + turn/2.0) +
						  crossrange * sin(filter->particles[i].theta + turn/2.0 + M_PI/2.0);
		}

		filter->particles[i].theta = carmen_normalize_theta(filter->particles[i].theta+turn);
	}
}


/* compute positions of laser points assuming robot pos is (0, 0, 0) */
static void
compute_positions_of_laser_points_in_the_robot_reference(carmen_localize_ackerman_particle_filter_p filter, double *laser_x, double *laser_y,
		carmen_localize_ackerman_map_p map, double *range, double angular_resolution, double first_beam_angle, double forward_offset, int num_readings,
		double laser_maxrange, int backwards)
{
	int i;
	double angle;
	for (i = 0; i < num_readings; i++)
	{
		angle = first_beam_angle + i * angular_resolution;

		laser_x[i] = (forward_offset + range[i] * cos(angle)) /
				map->config.resolution;
		laser_y[i] = (range[i] * sin(angle)) / map->config.resolution;

		if (backwards)
		{
			laser_x[i] = -laser_x[i];
			laser_y[i] = -laser_y[i];
		}
		if ((i % filter->param->laser_skip) == 0 &&
				range[i] < filter->param->max_range &&
				range[i] < laser_maxrange)
			filter->laser_mask[i] = 1;
		else
			filter->laser_mask[i] = 0;
	}
}


/* compute weight of each laser reading */
static void
compute_weight_of_each_laser_reading_using_local_map(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_p map,
		double *laser_x, double *laser_y, int num_readings)
{
	int i, j, x, y, robot_x, robot_y;
	double p_x, p_y, ctheta, stheta;

	double log_small_prob = log(filter->param->tracking_beam_minlikelihood);

	for (i = 0; i < filter->param->num_particles; i++)
	{
		filter->particles[i].weight = 0.0;
		p_x = (filter->particles[i].x - map->config.x_origin) / map->config.resolution;
		p_y = (filter->particles[i].y - map->config.y_origin) / map->config.resolution;

		ctheta = cos(filter->particles[i].theta);
		stheta = sin(filter->particles[i].theta);

		for (j = 0; j < num_readings; j += filter->param->laser_skip)
		{
			if (filter->laser_mask[j])
			{
				x = (p_x + laser_x[j] * ctheta - laser_y[j] * stheta);
				y = (p_y + laser_x[j] * stheta + laser_y[j] * ctheta);
				robot_x = p_x;
				robot_y = p_y;
				if (x < 0 || y < 0 || x >= map->config.x_size || y >= map->config.y_size || map->carmen_map.map[x][y] == -1)
					filter->temp_weights[i][j] = log_small_prob;
				else if (filter->param->constrain_to_map &&
					 (robot_x < 0 || robot_y < 0 || robot_x >= map->config.x_size || robot_y >= map->config.y_size || map->carmen_map.map[robot_x][robot_y] > filter->param->occupied_prob))
					filter->temp_weights[i][j] = log_small_prob;
				else
					filter->temp_weights[i][j] = map->prob[x][y];
			}
		}
	}
}


/* ignore laser readings that are improbable in a large fraction
       		  of the particles */
static void
ignore_some_laser_readings(carmen_localize_ackerman_particle_filter_p filter, int *count, int num_readings)
{
	int i, j;

	double log_min_wall_prob = log(filter->param->tracking_beam_minlikelihood);

	memset(count, 0, num_readings * sizeof(int));

	for (i = 0; i < filter->param->num_particles; i++)
		for (j = 0; j < num_readings; j += filter->param->laser_skip)
			if (filter->laser_mask[j] && filter->temp_weights[i][j] < log_min_wall_prob)
				count[j]++;

	for (i = 0; i < num_readings; i++)
		if (filter->laser_mask[i] && count[i] / (double) filter->param->num_particles > filter->param->outlier_fraction)
			filter->laser_mask[i] = 0;
}


/* add log probabilities to particle weights*/
void
add_log_probabilities_to_particle_weights(carmen_localize_ackerman_particle_filter_p filter, int num_readings)
{
	int i, j;

	for (i = 0; i < filter->param->num_particles; i++)
		for (j = 0; j < num_readings; j += filter->param->laser_skip)
			if (filter->laser_mask[j])
				filter->particles[i].weight += filter->temp_weights[i][j];
}



static void
change_size(double **laser_x, double **laser_y, int **count, int num_readings)
{
	static int _num_readings = 0;
	if (_num_readings != num_readings)
	{
		free(*laser_x);
		free(*laser_y);
		free(*count);

		*laser_x = (double *)calloc(num_readings, sizeof(double));
		carmen_test_alloc(*laser_x);
		*laser_y = (double *)calloc(num_readings, sizeof(double));
		carmen_test_alloc(*laser_y);
		*count = (int *)calloc(num_readings, sizeof(int));
		carmen_test_alloc(*count);

		_num_readings = num_readings;
	}
}


/* incorporate a single laser scan into the particle filter */
void 
carmen_localize_ackerman_incorporate_laser(carmen_localize_ackerman_particle_filter_p filter,
		carmen_localize_ackerman_map_p map, int num_readings,
		double *range, double forward_offset,
		double angular_resolution,
		double laser_maxrange,
		double first_beam_angle,
		int backwards)
{
	static double *laser_x = NULL, *laser_y = NULL;
	static int *count = NULL;

	change_size(&laser_x, &laser_y, &count, num_readings);

	/* compute the correct laser_skip */
	if (filter->param->laser_skip <= 0) 
	{
		filter->param->laser_skip = floor(filter->param->integrate_angle / angular_resolution);
	}

	compute_positions_of_laser_points_in_the_robot_reference(filter, laser_x, laser_y, map, range, angular_resolution, first_beam_angle,
			forward_offset, num_readings, laser_maxrange, backwards);

	/* test for global mode */
	//filter->converged = converged_test(filter);

//	if (filter->converged)
//	{
//		compute_weight_of_each_laser_reading_using_global_map(filter, map, laser_x, laser_y, num_readings);
//	}
//	else
	{
		compute_weight_of_each_laser_reading_using_local_map(filter, map, laser_x, laser_y, num_readings);

		/* ignore laser readings that are improbable in a large fraction
		       		  of the particles */
		ignore_some_laser_readings(filter, count, num_readings);

		add_log_probabilities_to_particle_weights(filter, num_readings);
	}
}


static void
velodyne_resample(carmen_localize_ackerman_particle_filter_p filter)
{
	static double *cumulative_sum = NULL;
	static carmen_localize_ackerman_particle_ipc_p temp_particles = NULL;
	static int num_particles = 0;

	carmen_localize_ackerman_particle_ipc_p aux;
	int i, which_particle;
	double weight_sum;
	double position, step_size;

	// alloc vectors if necessary
	if (num_particles != filter->param->num_particles)
	{
		if (temp_particles != NULL)
			free(temp_particles);
		if (cumulative_sum != NULL)
			free(cumulative_sum);

		/* Allocate memory necessary for resampling */
		cumulative_sum = (double *) calloc(filter->param->num_particles, sizeof(double));
		carmen_test_alloc(cumulative_sum);
		temp_particles = (carmen_localize_ackerman_particle_ipc_p) calloc(filter->param->num_particles, sizeof(carmen_localize_ackerman_particle_ipc_t));
		carmen_test_alloc(temp_particles);

		num_particles = filter->param->num_particles;
	}

	// Particles weights here are probabilities
	weight_sum = 0.0;	
	for (i = 0; i < filter->param->num_particles; i++)
	{	
		weight_sum += filter->particles[i].weight;
		cumulative_sum[i] = weight_sum;
	}

	/* choose random starting position for low-variance walk */
	position = carmen_uniform_random(0.0, weight_sum);
	step_size = weight_sum / (double) filter->param->num_particles;
	which_particle = 0;

	/* draw num_particles random samples */
	double even_weight = filter->param->num_particles;
	for (i = 0; i < filter->param->num_particles; i++)
	{
		while (position > cumulative_sum[which_particle])
			which_particle++;

		temp_particles[i] = filter->particles[which_particle];
		temp_particles[i].weight = 1.0 / even_weight; // reinitialize for the next cycle

		position += step_size;
		if (position > weight_sum)
		{
			position -= weight_sum;
			which_particle = 0;
		}
	}

	/* Switch particle pointers */
	aux = filter->particles;
	filter->particles = temp_particles;
	temp_particles = aux;
}


/* resample particle filter */
void
carmen_localize_ackerman_resample(carmen_localize_ackerman_particle_filter_p filter)
{
	static double *cumulative_sum = NULL;
	static carmen_localize_ackerman_particle_ipc_p temp_particles = NULL;
	static int num_particles = 0;

	carmen_localize_ackerman_particle_ipc_p aux;
	int i, which_particle;
	double weight_sum;
	double position, step_size, max_weight;

	// alloc vectors if necessary
	if (num_particles != filter->param->num_particles)
	{
		free(temp_particles);
		free(cumulative_sum);

		/* Allocate memory necessary for resampling */
		cumulative_sum = (double *)calloc(filter->param->num_particles, sizeof(double));
		carmen_test_alloc(cumulative_sum);
		temp_particles = (carmen_localize_ackerman_particle_ipc_p) calloc(filter->param->num_particles, sizeof(carmen_localize_ackerman_particle_ipc_t));
		carmen_test_alloc(temp_particles);

		num_particles = filter->param->num_particles;
	}

	weight_sum = 0.0;
	max_weight = filter->particles[0].weight;
	/* change log weights back into probabilities */
	for (i = 0; i < filter->param->num_particles; i++)
	{
		if (filter->particles[i].weight > max_weight)
			max_weight = filter->particles[i].weight;
	}

	for (i = 0; i < filter->param->num_particles; i++)
	{
		//change log weights to probabilities weights
		filter->particles[i].weight = exp(filter->particles[i].weight - max_weight);

		/* Sum the weights of all of the particles */
		weight_sum += filter->particles[i].weight;
		cumulative_sum[i] = weight_sum;
	}

	/* choose random starting position for low-variance walk */
	position = carmen_uniform_random(0, weight_sum);
	step_size = weight_sum / (double)filter->param->num_particles;
	which_particle = 0;

	/* draw num_particles random samples */
	for(i = 0; i < filter->param->num_particles; i++)
	{
		position += step_size;

		if (position > weight_sum)
		{
			position -= weight_sum;
			which_particle = 0;
		}

		while(position > cumulative_sum[which_particle])
			which_particle++;

		temp_particles[i] = filter->particles[which_particle];
		temp_particles[i].weight = 0.5;
	}

	/* Switch particle pointers */
	aux = filter->particles;
	filter->particles = temp_particles;
	temp_particles = aux;
}


static void
correction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_p map,
		carmen_laser_laser_config_t *laser_config, int num_readings,
		double *range, double forward_offset, int backwards)
{
	if (filter->param->use_sensor)
	{
		/* incorporate the laser scan */
		carmen_localize_ackerman_incorporate_laser(filter, map, num_readings,
				range, forward_offset,
				laser_config->angular_resolution,
				laser_config->maximum_range,
				laser_config->start_angle,
				backwards);

		/* check if it is time to resample */
		if (filter->distance_travelled >= filter->param->update_distance)
		{
			carmen_localize_ackerman_resample(filter);
			filter->distance_travelled = 0;
			filter->initialized = 1;
		}
	}
}


/* incorporate a robot laser reading */

void carmen_localize_ackerman_run(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_p map,
		carmen_robot_ackerman_laser_message *laser, double forward_offset,
		int backwards, carmen_base_ackerman_odometry_message *odometry, double distance_between_front_and_rear_axles)
{

	if (!filter->initialized)
		return;

	/* incorporate the laser position stamp */

	/* Prediction */
	carmen_localize_ackerman_incorporate_odometry(filter, odometry->v, odometry->phi, distance_between_front_and_rear_axles, laser->timestamp - filter->last_timestamp);

	/* Correction */
	correction(filter, map, &(laser->config), laser->num_readings, laser->range, forward_offset, backwards);

	filter->last_timestamp = laser->timestamp;

}


void
carmen_localize_ackerman_run_with_raw_laser(
		carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_p map,
		carmen_laser_laser_message *laser, carmen_base_ackerman_odometry_message *odometry,
		double forward_offset, double distance_between_front_and_rear_axles)
{
	if (!filter->initialized)
		return;

	/* Prediction */
	if (filter->param->prediction_type == 2)
		carmen_die("In navigate mode, the carmen.ini cannot be set with localize_ackerman_prediction_type = 2\n");
	else if (filter->param->prediction_type == 1)
		carmen_localize_ackerman_incorporate_velocity_odometry(filter, odometry->v, odometry->phi, distance_between_front_and_rear_axles, laser->timestamp - filter->last_timestamp);
	else // differential
		carmen_localize_ackerman_incorporate_odometry(filter, odometry->v, odometry->phi, distance_between_front_and_rear_axles, laser->timestamp - filter->last_timestamp);

	filter->last_timestamp = laser->timestamp;
	/* Correction */
	correction(filter, map, &(laser->config), laser->num_readings, laser->range, forward_offset, 0);
}


cell_coords_t
calc_global_cell_coordinate(cell_coords_t *local, carmen_map_config_t *local_map_config, carmen_vector_2D_t *robot_position,
		double sin_theta, double cos_theta)
{
	cell_coords_t global;
	double dx, dy, dxg, dyg;

	dx = ((double) local->x - (double) local_map_config->x_size * 0.5);
	dy = ((double) local->y - (double) local_map_config->y_size * 0.5);
	dxg = dx * cos_theta - dy * sin_theta;
	dyg = dx * sin_theta + dy * cos_theta;

	global.x = (int) round(dxg + robot_position->x / local_map_config->resolution);
	global.y = (int) round(dyg + robot_position->y / local_map_config->resolution);

	return (global);
}


void
calc_global_cell_coordinate_fast(cell_coords_t *global, cell_coords_t local,
		double map_center_x, double map_center_y,
		double robot_position_in_the_map_x, double robot_position_in_the_map_y,
		double sin_theta, double cos_theta)
{
	double dx = (double) local.x - map_center_x;
	double dy = (double) local.y - map_center_y;
	double dxg = dx * cos_theta - dy * sin_theta;
	double dyg = dx * sin_theta + dy * cos_theta;

	global->x = (int) round(dxg + robot_position_in_the_map_x);
	global->y = (int) round(dyg + robot_position_in_the_map_y);
}


void
calc_posible_particle_position(carmen_localize_ackerman_particle_filter_p filter, carmen_pose_3D_t pose)
{
	double sin_theta, cos_theta;
	double x, y, theta;
	int i = 0;

	theta = pose.orientation.yaw;
	sin_theta = sin(theta);
	cos_theta = cos(theta);

	for(x = -1.0; x <= 1.0; x += 0.5)
	{
		for(y = -1.0; y <= 1.0; y += 0.5, i++)
		{
			if (i == filter->param->num_particles)
				return;

			filter->particles[i].x = pose.position.x + x * cos_theta - y * sin_theta;
			filter->particles[i].y = pose.position.y + x * sin_theta + y * cos_theta;
			//filter->particles[i].theta = theta;
		}
	}
}


static double
pearson_correlation_correction(carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_map, carmen_localize_ackerman_particle_ipc_t *particle, double map_mean)
{
	int i;
	cell_coords_t local, global;
	double local_diff, global_diff;
	double sum_global_diff_sqr = 0.0, sum_local_diff_sqr = 0.0;
	double sum_diff = 0.0;
	double sin_theta, cos_theta;
	carmen_vector_2D_t robot_position;

	sin_theta = sin(particle->theta);
	cos_theta = cos(particle->theta);
	robot_position.x = particle->x - global_map->config.x_origin;
	robot_position.y = particle->y - global_map->config.y_origin;


	for (i = 0; i < local_map->number_of_known_points_on_the_map; i++)
	{
		local.x = local_map->coord_x[i];
		local.y = local_map->coord_y[i];
		global = calc_global_cell_coordinate(&local, &local_map->config, &robot_position, sin_theta, cos_theta);

		if (global.x >= 0 && global.y >= 0 && global.x < global_map->config.x_size && global.y < global_map->config.y_size)
		{
			local_diff = local_map->value[i] - map_mean;
			if (global_map->carmen_map.map[global.x][global.y] >= 0.0)
			{				
				global_diff = global_map->carmen_map.map[global.x][global.y] - map_mean;
				sum_diff += local_diff * global_diff;
				sum_global_diff_sqr += global_diff * global_diff;
			}
			sum_local_diff_sqr += local_diff * local_diff;
		}
	}

	double weight = sum_diff / sqrt(sum_global_diff_sqr * sum_local_diff_sqr);

	if (isnan(weight) || weight < 0.0)
		weight = 0.0;

	return (weight);
}


void
inner_product_between_maps(double *inner_product, double *norm_local_map, double *norm_global_map, carmen_map_t *global_map, carmen_compact_map_t *local_map, carmen_localize_ackerman_particle_ipc_t *particle)
{
	int i;
	cell_coords_t local, global;
	double sum_global = 0.0, sum_local = 0.0;
	double sum = 0.0;
	double sin_theta, cos_theta;
	carmen_vector_2D_t robot_position;

	sin_theta = sin(particle->theta);
	cos_theta = cos(particle->theta);
	double map_center_x = (double) local_map->config.x_size * 0.5;
	double map_center_y = (double) local_map->config.y_size * 0.5;
	robot_position.x = particle->x - global_map->config.x_origin;
	robot_position.y = particle->y - global_map->config.y_origin;
	double robot_position_in_the_map_x = robot_position.x / local_map->config.resolution;
	double robot_position_in_the_map_y = robot_position.y / local_map->config.resolution;

	for (i = 0; i < local_map->number_of_known_points_on_the_map; i += 8)
	{
		local.x = local_map->coord_x[i];
		local.y = local_map->coord_y[i];

		calc_global_cell_coordinate_fast(&global, local, map_center_x, map_center_y,
					robot_position_in_the_map_x, robot_position_in_the_map_y, sin_theta, cos_theta);

		if (global.x >= 0 && global.y >= 0 && global.x < global_map->config.x_size && global.y < global_map->config.y_size)
		{
			if (global_map->map[global.x][global.y] <= 0.001)
			{
				sum_global += 2; // Alberto: @@@ Por que?
			}
			else
			{
				sum += local_map->value[i] * global_map->map[global.x][global.y];
				sum_global += global_map->map[global.x][global.y] * global_map->map[global.x][global.y];
			}
			sum_local += local_map->value[i] * local_map->value[i];

		}
	}

	*inner_product = sum;
	*norm_global_map = sum_global;
	*norm_local_map = sum_local;
}


static double
cosine_correlation_correction_with_likelihood_map(carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_map, carmen_localize_ackerman_particle_ipc_t *particle)
{
	int i;
	cell_coords_t local, global;
	double sum_global = 0.0, sum_local = 0.0;
	double sum = 0.0;
	double sin_theta, cos_theta;
	carmen_vector_2D_t robot_position;

	sin_theta = sin(particle->theta);
	cos_theta = cos(particle->theta);
	robot_position.x = particle->x - global_map->config.x_origin;
	robot_position.y = particle->y - global_map->config.y_origin;


	for (i = 0; i < local_map->number_of_known_points_on_the_map; i++)
	{
		local.x = local_map->coord_x[i];
		local.y = local_map->coord_y[i];
		global = calc_global_cell_coordinate(&local, &local_map->config, &robot_position, sin_theta, cos_theta);

		if (global.x >= 0 && global.y >= 0 && global.x < global_map->config.x_size && global.y < global_map->config.y_size)
		{
			sum += local_map->value[i] * exp(global_map->prob[global.x][global.y]);
			sum_local += local_map->value[i] * local_map->value[i];
			sum_global += exp(global_map->prob[global.x][global.y]) * exp(global_map->prob[global.x][global.y]);
		}
	}

	double weight = sum / (sqrt(sum_local) * sqrt(sum_global));

	if (isnan(weight) || weight < 0.0)
		weight = 0.0;

	return (weight);
}


static double
get_particle_max_weight(carmen_localize_ackerman_particle_filter_p filter)
{
	double max_weight = filter->particles[0].weight;

	int i;
	for (i = 1; i < filter->param->num_particles; i++)
	{
		if (filter->particles[i].weight > max_weight)
			max_weight = filter->particles[i].weight;
	}

	return max_weight;
}


static void
convert_particles_log_odd_weights_to_prob(carmen_localize_ackerman_particle_filter_p filter)
{
	double max_weight = get_particle_max_weight(filter);

//	FILE *caco;
//	caco = fopen("caco11.txt", "a");

	for (int i = 0; i < filter->param->num_particles; i++)
	{
		double weight = filter->particles[i].weight;

		if (filter->param->use_log_odds)
			filter->particles[i].weight = carmen_prob_models_log_odds_to_probabilistic(weight - max_weight);
		else
			filter->particles[i].weight = exp(weight - max_weight);

//		fprintf(caco, "%04d, p %.15lf, mw %lf, w %lf\n", i, filter->particles[i].weight, max_weight, weight);
	}
//	fclose(caco);
}


void
normalize_particles_map_matching(carmen_localize_ackerman_particle_filter_p filter)
{
	double max = -1000000000000000000.0;
	double min = 100000000000000000000000.0;

	int i;
	for (i = 0; i < filter->param->num_particles; i++)
	{
		if (max < filter->particles[i].weight)
			max = filter->particles[i].weight;

		if(min > filter->particles[i].weight)
			min = filter->particles[i].weight;
	}

	for (i = 0; i < filter->param->num_particles; i++)
	{
		if (max - min != 0.0)
			filter->particles[i].weight = (filter->particles[i].weight - min) / (max - min);
		else
			filter->particles[i].weight = 0.000001;
	}
}


static void
cosine_correction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_map)
{

	double w;
	double inner_product, norm_local_map, norm_global_map;

	int i;
	for (i = 0; i < filter->param->num_particles; i++)
	{
		inner_product_between_maps(&inner_product, &norm_local_map, &norm_global_map, &global_map->carmen_map, local_map, &(filter->particles[i]));
		if ((norm_local_map > 0.001) && (norm_global_map > 0.001))
			w = inner_product / (sqrt(norm_local_map) * sqrt(norm_global_map));
		else
			w = -1.0;
		filter->particles[i].weight = 1.0 + w;
	}
	normalize_particles_map_matching(filter);
	//convert_particles_log_weights_to_prob(filter);
}


static double
hausdoff_distance(carmen_map_t *global_map, carmen_compact_map_t *local_map, carmen_localize_ackerman_particle_ipc_t *particle)
{
	int i, j;
	cell_coords_t localA, localB, globalA, globalB;
	double sin_theta, cos_theta;
	double HAB = 0.0, HBA = 0.0;
	carmen_vector_2D_t robot_position;

	sin_theta = sin(particle->theta);
	cos_theta = cos(particle->theta);
	robot_position.x = particle->x - global_map->config.x_origin;
	robot_position.y = particle->y - global_map->config.y_origin;
	double min;
	double max = -1.0;

	for (i = 0; i < local_map->number_of_known_points_on_the_map; i++)
	{
		localA.x = local_map->coord_x[i];
		localA.y = local_map->coord_y[i];
		globalA = calc_global_cell_coordinate(&localA, &local_map->config, &robot_position, sin_theta, cos_theta);

		if (globalA.x >= 0 && globalA.y >= 0 && globalA.x < global_map->config.x_size && globalA.y < global_map->config.y_size)
		{
			min = 100000;

			for (j = 0; j < local_map->number_of_known_points_on_the_map; j++)
			{
				localB.x = local_map->coord_x[j];
				localB.y = local_map->coord_y[j];
				globalB = calc_global_cell_coordinate(&localB, &local_map->config, &robot_position, sin_theta, cos_theta);

				if (globalB.x >= 0 && globalB.y >= 0 && globalB.x < global_map->config.x_size && globalB.y < global_map->config.y_size)
				{
					if (global_map->map[globalB.x][globalB.y] > 0.5)
					{
						double d = sqrt(pow(globalA.x - globalB.x, 2) + pow(globalA.y - globalB.y,2));
						if (min > d)
							min = d;
					}
				}
			}
			if (max < min)
				max = min;
		}
	}

	HAB = max;
	max = -1.0;

	for (i = 0; i < local_map->number_of_known_points_on_the_map; i++)
	{
		localB.x = local_map->coord_x[i];
		localB.y = local_map->coord_y[i];
		globalB = calc_global_cell_coordinate(&localB, &local_map->config, &robot_position, sin_theta, cos_theta);
		if (globalB.x >= 0 && globalB.y >= 0 && globalB.x < global_map->config.x_size && globalB.y < global_map->config.y_size)
		{
			if (global_map->map[globalB.x][globalB.y] > 0.5)
			{
				min = 100000;

				for (j = 0; j < local_map->number_of_known_points_on_the_map; j++)
				{
					localA.x = local_map->coord_x[j];
					localA.y = local_map->coord_y[j];
					globalA = calc_global_cell_coordinate(&localA, &local_map->config, &robot_position, sin_theta, cos_theta);

					if (globalA.x >= 0 && globalA.y >= 0 && globalA.x < global_map->config.x_size && globalA.y < global_map->config.y_size)
					{
						double d = sqrt(pow(globalA.x - globalB.x, 2) + pow(globalA.y - globalB.y,2));
						if (min > d)
							min = d;
					}
				}
				if (max < min)
					max = min;
			}
		}
	}

	HBA = max;

	return MAX(HBA, HAB);
}


void
hausdoff_distance_correction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_map)
{
	double w;

	int i;
	for (i = 0; i < filter->param->num_particles; i++)
	{
		w = hausdoff_distance(&global_map->carmen_map, local_map, &(filter->particles[i]));
		filter->particles[i].weight = -w;
	}

	 get_particle_max_weight(filter);
	 convert_particles_log_odd_weights_to_prob(filter);
}


static double
map_particle_correction(carmen_localize_ackerman_map_t *localize_map, carmen_compact_map_t *local_map,
		carmen_localize_ackerman_particle_filter_p filter, int particle_index)
{
	cell_coords_t local_cell, global_cell;
	carmen_vector_2D_t robot_position;

	carmen_localize_ackerman_particle_ipc_t *particle = &(filter->particles[particle_index]);

	double sin_theta = sin(particle->theta);
	double cos_theta = cos(particle->theta);
	double map_center_x = (double) local_map->config.x_size * 0.5;
	double map_center_y = (double) local_map->config.y_size * 0.5;
	robot_position.x = particle->x - localize_map->config.x_origin;
	robot_position.y = particle->y - localize_map->config.y_origin;
	double robot_position_in_the_map_x = robot_position.x / local_map->config.resolution;
	double robot_position_in_the_map_y = robot_position.y / local_map->config.resolution;

	double small_log_odds;
	if (filter->param->use_log_odds)
		small_log_odds = log(filter->param->tracking_beam_minlikelihood / (1.0 - filter->param->tracking_beam_minlikelihood));
	else
		small_log_odds = log(filter->param->tracking_beam_minlikelihood);

	double particle_weight = 0.0;
	for (int i = 0; i < local_map->number_of_known_points_on_the_map; i++)
	{
		local_cell.x = local_map->coord_x[i];
		local_cell.y = local_map->coord_y[i];

		calc_global_cell_coordinate_fast(&global_cell, local_cell, map_center_x, map_center_y,
					robot_position_in_the_map_x, robot_position_in_the_map_y, sin_theta, cos_theta);

		if (global_cell.x >= 0 && global_cell.y >= 0 && global_cell.x < localize_map->config.x_size && global_cell.y < localize_map->config.y_size)
			particle_weight += localize_map->prob[global_cell.x][global_cell.y];
		else
			particle_weight += small_log_odds;
	}

	return (particle_weight);
}

//#include <carmen/global_graphics.h>

static void
compute_particles_weights_with_outlier_rejection(carmen_localize_ackerman_map_t *localize_map,
                                                 carmen_compact_map_t *local_map,
                                                 carmen_localize_ackerman_particle_filter_p filter)
{
	double map_center_x = (double) local_map->config.x_size * 0.5;
	double map_center_y = (double) local_map->config.y_size * 0.5;
	double small_log_weight;

	if (filter->param->use_log_odds)
		small_log_weight = log(filter->param->tracking_beam_minlikelihood / (1.0 - filter->param->tracking_beam_minlikelihood));
	else
		small_log_weight = log(filter->param->tracking_beam_minlikelihood);

	int *count = NULL;
	int num_readings = local_map->number_of_known_points_on_the_map;

	if (num_readings > 0)
		count = (int *) calloc(num_readings, sizeof(int));

//	carmen_mapper_virtual_laser_message virtual_laser_message;
//	if (num_readings > 0)
//	{
//		virtual_laser_message.positions = (carmen_position_t *) calloc(num_readings, sizeof(carmen_position_t));
//		virtual_laser_message.colors = (char *) calloc(num_readings, sizeof(char));
//	}

//	FILE *p = fopen("p.txt", "a");
	for (int i = 0; i < filter->param->num_particles; i++)
	{
		carmen_localize_ackerman_particle_ipc_t *particle = &(filter->particles[i]);

		double sin_theta = sin(particle->theta);
		double cos_theta = cos(particle->theta);
		double robot_position_in_the_map_x = (particle->x - localize_map->config.x_origin) / local_map->config.resolution;
		double robot_position_in_the_map_y = (particle->y - localize_map->config.y_origin) / local_map->config.resolution;

//		fprintf(p, "%lf %lf\n", robot_position_in_the_map_x, robot_position_in_the_map_y);

		particle->weight = 0.0;
		for (int laser_reading = 0; laser_reading < num_readings; laser_reading++)
		{
			cell_coords_t local_cell;
			local_cell.x = local_map->coord_x[laser_reading];
			local_cell.y = local_map->coord_y[laser_reading];

			cell_coords_t global_cell;
			calc_global_cell_coordinate_fast(&global_cell, local_cell, map_center_x, map_center_y,
						robot_position_in_the_map_x, robot_position_in_the_map_y, sin_theta, cos_theta);

			if (global_cell.x >= 0 && global_cell.y >= 0 && global_cell.x < localize_map->config.x_size && global_cell.y < localize_map->config.y_size)
				filter->temp_weights[i][laser_reading] = localize_map->prob[global_cell.x][global_cell.y];
			else
				filter->temp_weights[i][laser_reading] = small_log_weight;

			if (filter->temp_weights[i][laser_reading] <= small_log_weight) // Provavelmente bateu no vazio
				count[laser_reading] += 1;

//			virtual_laser_message.positions[laser_reading].x = global_cell.x * localize_map->config.resolution + localize_map->config.x_origin;
//			virtual_laser_message.positions[laser_reading].y = global_cell.y * localize_map->config.resolution + localize_map->config.y_origin;
//			virtual_laser_message.colors[laser_reading] = CARMEN_RED;
		}
	}
//	fprintf(p, "\n");
//	fclose(p);

	for (int laser_reading = 0; laser_reading < num_readings; laser_reading++)
	{
		if (((double) count[laser_reading] / (double) filter->param->num_particles) < filter->param->outlier_fraction)
			for (int i = 0; i < filter->param->num_particles; i++)
				filter->particles[i].weight += filter->temp_weights[i][laser_reading];
//		else
//			virtual_laser_message.colors[laser_reading] = CARMEN_GREEN;
	}

	for (int i = 0; i < filter->param->num_particles; i++)
		filter->particles[i].weight *= filter->param->particles_normalize_factor;

	if (num_readings > 0)
		free(count);

//	if (num_readings > 0)
//	{
//		virtual_laser_message.num_positions = num_readings;
//		virtual_laser_message.host = carmen_get_host();
//		carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());
//		free(virtual_laser_message.positions);
//		free(virtual_laser_message.colors);
//	}
}


static void
cosine_correction_with_remission_map_and_log_likelihood(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_mean_remission_map, carmen_compact_map_t *local_map)
{

	double min_weight = 0.000001, w, w2;
	double inner_product_remission, norm_local_remission_map, norm_global_remission_map;
	double inner_product = 0.0, norm_local_map = 0.0, norm_global_map = 0.0;

	int i;

	for (i = 0; i < filter->param->num_particles; i++)
	{
		w2 = map_particle_correction(global_map, local_map, filter, i);
		inner_product_between_maps(&inner_product_remission, &norm_local_remission_map, &norm_global_remission_map, &global_map->carmen_mean_remission_map, local_mean_remission_map, &(filter->particles[i]));
		//inner_product_between_maps(&inner_product, &norm_local_map, &norm_global_map, &global_map->carmen_map, local_map, &(filter->particles[i]));
		inner_product = inner_product + inner_product_remission;
		norm_local_map = norm_local_map + norm_local_remission_map;
		norm_global_map = norm_global_map + norm_global_remission_map;
		w = (inner_product) / (sqrt(norm_local_map) * sqrt(norm_global_map));
		w = 1.0 - (w > min_weight ? w : min_weight);
		filter->particles[i].weight = w2 + (-(w * w) * 100.0);
	}
	convert_particles_log_odd_weights_to_prob(filter);
}


static void
cosine_correction_with_remission_map_and_grid_map(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_mean_remission_map, carmen_compact_map_t *local_map)
{

	double min_weight = 0.000001, w;
	double inner_product_remission, norm_local_remission_map, norm_global_remission_map;
	double inner_product = 0.0, norm_local_map = 0.0, norm_global_map = 0.0;

	int i;
#pragma omp parallel for
	for (i = 0; i < filter->param->num_particles; i++)
	{
		inner_product_between_maps(&inner_product_remission, &norm_local_remission_map, &norm_global_remission_map, &global_map->carmen_mean_remission_map, local_mean_remission_map, &(filter->particles[i]));
		inner_product_between_maps(&inner_product, &norm_local_map, &norm_global_map, &global_map->carmen_map, local_map, &(filter->particles[i]));
		inner_product = inner_product + inner_product_remission;
		norm_local_map = norm_local_map + norm_local_remission_map;
		norm_global_map = norm_global_map + norm_global_remission_map;
		w = (inner_product) / (sqrt(norm_local_map) * sqrt(norm_global_map));
		w = 1.0 - (w > min_weight ? w : min_weight);
		filter->particles[i].weight = (-(w * w) * 100.0);
	}
	convert_particles_log_odd_weights_to_prob(filter);
}


double
mahalanobis_distance_between_maps(carmen_map_t *global_mean_map, carmen_map_t *global_variance_map, carmen_compact_map_t *local_mean_map, carmen_localize_ackerman_particle_ipc_t *particle)
{
	int i;
	cell_coords_t local, global;
	double sum = 0.0;
	double sin_theta, cos_theta;
	carmen_vector_2D_t robot_position;

	sin_theta = sin(particle->theta);
	cos_theta = cos(particle->theta);
	robot_position.x = particle->x - global_mean_map->config.x_origin;
	robot_position.y = particle->y - global_mean_map->config.y_origin;


	for (i = 0; i < local_mean_map->number_of_known_points_on_the_map; i++)
	{
		local.x = local_mean_map->coord_x[i];
		local.y = local_mean_map->coord_y[i];
		global = calc_global_cell_coordinate(&local, &local_mean_map->config, &robot_position, sin_theta, cos_theta);

		if (global.x >= 0 && global.y >= 0 && global.x < global_mean_map->config.x_size && global.y < global_mean_map->config.y_size)
			sum += pow(local_mean_map->value[i] - global_mean_map->map[global.x][global.y], 2) / global_variance_map->map[global.x][global.y];
	}

	if (sum > 0.0)
		return sqrt(sum);
	return 0.0000001;
}


void
mahalanobis_distance_correction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_mean_remission_map)
{

	double  w, max_weight;

	int i;
	for (i = 0; i < filter->param->num_particles; i++)
	{
		w = mahalanobis_distance_between_maps(&global_map->carmen_mean_remission_map, &global_map->carmen_variance_remission_map, local_mean_remission_map, &(filter->particles[i]));
		filter->particles[i].weight = w;
	}

	max_weight = get_particle_max_weight(filter);

	for (i = 0; i < filter->param->num_particles; i++)
		filter->particles[i].weight = (filter->particles[i].weight/max_weight);
}


static void
cosine_correction_with_remission_map(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_mean_remission_map)
{

	double min_weight = 0.000001, w;
	double inner_product, norm_local_map, norm_global_map;
	int i;
//#pragma omp parallel for
	for (i = 0; i < filter->param->num_particles; i++)
	{
		inner_product_between_maps(&inner_product, &norm_local_map, &norm_global_map, &global_map->carmen_mean_remission_map, local_mean_remission_map, &(filter->particles[i]));
		w = inner_product / (sqrt(norm_local_map) * sqrt(norm_global_map));
		w = 1.0 - (w > min_weight ? w : min_weight);
		filter->particles[i].weight = (-(w * w) * 100.0);
	}
	convert_particles_log_odd_weights_to_prob(filter);
}


cell_coords_t
calc_global_cell_coordinate2(cell_coords_t *local, carmen_map_config_t *local_map_config, carmen_vector_2D_t *robot_position,
		double sin_theta, double cos_theta)
{
	cell_coords_t global;
	double dx, dy, dxg, dyg;

	dx = ((double) local->x);// - (double) local_map_config->x_size / 2.0);
	dy = ((double) local->y);// - (double) local_map_config->y_size / 2.0);
	dxg = dx * cos_theta - dy * sin_theta;
	dyg = dx * sin_theta + dy * cos_theta;

	global.x = (int) ((dxg + robot_position->x / local_map_config->resolution) + 0.5);
	global.y = (int) ((dyg + robot_position->y / local_map_config->resolution) + 0.5);

	return (global);
}


double
normalized_mutual_information(carmen_map_t *global_map, carmen_compact_map_t *local_map, carmen_localize_ackerman_particle_ipc_t *particle)
{
	int i, j;
	cell_coords_t local, global;
	double sin_theta, cos_theta;
	carmen_vector_2D_t robot_position;

	double hA[256];
	double hB[256];
	double hAB[256][256];

	double HA = 0.0, HB = 0.0, HAB = 0.0;
	double sum_A = 0.0, sum_B = 0.0, sum_AB = 0.0;

	memset(hA, 0, 256 * sizeof(double));
	memset(hB, 0, 256 * sizeof(double));

	for (i = 0; i < 256; i++)
		memset(hAB[i], 0, 256 * sizeof(double));

	sin_theta = sin(particle->theta);
	cos_theta = cos(particle->theta);
	robot_position.x = particle->x - global_map->config.x_origin;
	robot_position.y = particle->y - global_map->config.y_origin;

	for (i = 0; i < local_map->number_of_known_points_on_the_map; i++)
	{
		local.x = local_map->coord_x[i];
		local.y = local_map->coord_y[i];
		global = calc_global_cell_coordinate(&local, &local_map->config, &robot_position, sin_theta, cos_theta);

		int indexA = (int)(255 * local_map->value[i]);
		if (global.x >= 0 && global.y >= 0 && global.x < global_map->config.x_size && global.y < global_map->config.y_size)
		{
			int indexB = (int)(255 * (global_map->map[global.x][global.y]));

			if (global_map->map[global.x][global.y] > 0.0)
			{
				hAB[indexA][indexB] += 1.0;
				hB[indexB] += 1.0;
			}
		}
		hA[indexA] += 1.0;
	}

	for (i = 0; i < 256; i++)
	{
		sum_A += hA[i];
		sum_B += hB[i];
		for (j = 0; j < 256; j++)
			sum_AB += hAB[i][j];
	}

	for (i = 0; i < 256; i++)
	{
		if (hA[i] > 0.5)
		{
			hA[i] /= sum_A;
			HA += hA[i] * log2(hA[i]);
		}

		if (hB[i] > 0.5)
		{
			hB[i] /= sum_B;
			HB += hB[i] * log2(hB[i]);
		}
		for (j = 0; j < 256; j++)
		{
			if (hAB[i][j] > 0.5)
			{
				hAB[i][j] /= sum_AB;
				HAB += hAB[i][j] * log2(hAB[i][j]);
			}
		}
	}
	if (fabs(HAB) < 0.0000001)
		return 0.0;
	//printf("sumA %lf sumB %lf sumAB %lf\n", sum_A, sum_B, sum_AB);
	//printf("%HA %lf HB %lf HAB %lf %lf\n", HA, HB, HAB,((-HA) + (-HB)) / (-HAB));
	return ((-HA) + (-HB)) / (-HAB);
}


void
mutual_information_correction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_mean_remission_map)
{

	double w;
	int i;
	for (i = 0; i < filter->param->num_particles; i++)
	{
		w = normalized_mutual_information(&global_map->carmen_mean_remission_map, local_mean_remission_map, &(filter->particles[i]));
		//w = (w - 1.15) > 0.0 ? ((w - 1.15) * 10.0) : 0.0;
		w = 2.0 - w;
		//w = 1.0 - w / 2.0;

		filter->particles[i].weight = (-(w * w * 100.0));
	}
	convert_particles_log_odd_weights_to_prob(filter);
}


double
hamming_distance(carmen_map_t *global_map, carmen_compact_map_t *local_map, carmen_localize_ackerman_binary_map_t *binary_map, carmen_localize_ackerman_particle_ipc_t *particle)
{
	int i;
	cell_coords_t local1, local2, global1, global2;
	double sin_theta, cos_theta;
	carmen_vector_2D_t robot_position;
	int hamming_dist = 0;

	sin_theta = sin(particle->theta);
	cos_theta = cos(particle->theta);
	robot_position.x = particle->x - global_map->config.x_origin;
	robot_position.y = particle->y - global_map->config.y_origin;

	for (i = 0; i < binary_map->map_size - 1; i++)
	{
		local1.x = local_map->coord_x[binary_map->rand_position[i]];
		local1.y = local_map->coord_y[binary_map->rand_position[i]];
		local2.x = local_map->coord_x[binary_map->rand_position[i + 1]];
		local2.y = local_map->coord_y[binary_map->rand_position[i + 1]];

		global1 = calc_global_cell_coordinate(&local1, &local_map->config, &robot_position, sin_theta, cos_theta);
		global2 = calc_global_cell_coordinate(&local2, &local_map->config, &robot_position, sin_theta, cos_theta);

		if (global1.x >= 0 && global1.y >= 0 && global1.x < global_map->config.x_size && global1.y < global_map->config.y_size)
		{
			if (global2.x >= 0 && global2.y >= 0 && global2.x < global_map->config.x_size && global2.y < global_map->config.y_size)
			{
				if (global_map->map[global1.x][global1.y] > 0.0 && global_map->map[global2.x][global2.y] > 0.0)
				{
					int bin = global_map->map[global1.x][global1.y] > global_map->map[global2.x][global2.y] ? 1 : 0;
//					int bin = fabs(global_map->map[global1.x][global1.y] - global_map->map[global2.x][global2.y]) > 0.2 ? 1 : 0;
					hamming_dist += bin ^ binary_map->binary_map[i];
				}
				else
					hamming_dist += 1;
			}
		}
	}

	local1.x = local_map->coord_x[binary_map->rand_position[i]];
	local1.y = local_map->coord_y[binary_map->rand_position[i]];
	local2.x = local_map->coord_x[binary_map->rand_position[0]];
	local2.y = local_map->coord_y[binary_map->rand_position[0]];

	global1 = calc_global_cell_coordinate(&local1, &local_map->config, &robot_position, sin_theta, cos_theta);
	global2 = calc_global_cell_coordinate(&local2, &local_map->config, &robot_position, sin_theta, cos_theta);

	if (global1.x >= 0 && global1.y >= 0 && global1.x < global_map->config.x_size && global1.y < global_map->config.y_size)
	{
		if (global2.x >= 0 && global2.y >= 0 && global2.x < global_map->config.x_size && global2.y < global_map->config.y_size)
		{
			if (global_map->map[global1.x][global1.y] > 0.0 && global_map->map[global2.x][global2.y] > 0.0)
			{
//				int bin = fabs(global_map->map[global1.x][global1.y] - global_map->map[global2.x][global2.y]) > 0.2 ? 1 : 0;
				int bin = global_map->map[global1.x][global1.y] > global_map->map[global2.x][global2.y] ? 1 : 0;
				hamming_dist += bin ^ binary_map->binary_map[i];
			}
			else
				hamming_dist += 1;
		}
	}

	return (double)hamming_dist / local_map->number_of_known_points_on_the_map;
}


void
hamming_distance_between_remission_maps(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_map, carmen_localize_ackerman_binary_map_t *binary_map)
{
	int i;
	double w;
	for (i = 0; i < filter->param->num_particles; i++)
	{
		w = hamming_distance(&global_map->carmen_mean_remission_map, local_map, binary_map, &(filter->particles[i]));
//		w = 1.0 - w;
		filter->particles[i].weight = ((-(w * w) * 100.0));
	}
	convert_particles_log_odd_weights_to_prob(filter);
}


static void
cosine_correction_with_likelihood_map(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_map)
{

	double min_weight = 0.000001;
	double w;

	int i;
	for (i = 0; i < filter->param->num_particles; i++)
	{
		w = cosine_correlation_correction_with_likelihood_map(global_map, local_map, &(filter->particles[i]));
		w = 1.0 - (w > min_weight ? w : min_weight);
		filter->particles[i].weight = exp(-(w * w) * 100.0);
	}
}


static double
calc_map_mean(carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_map, carmen_localize_ackerman_particle_ipc_t *particle)
{
	int i;
	cell_coords_t local, global;
	double map_mean = 0.0;
	double sin_theta, cos_theta;
	carmen_vector_2D_t robot_position;

	sin_theta = sin(particle->theta);
	cos_theta = cos(particle->theta);
	robot_position.x = particle->x;
	robot_position.y = particle->y;

	for (i = 0; i < local_map->number_of_known_points_on_the_map; i++)
	{
		local.x = local_map->coord_x[i];
		local.y = local_map->coord_y[i];
		global = calc_global_cell_coordinate(&local, &local_map->config, &robot_position, sin_theta, cos_theta);

		if (global.x >= 0 && global.y >= 0 && global.x < global_map->config.x_size && global.y < global_map->config.y_size)
		{
			if (global_map->carmen_map.map[global.x][global.y] >= 0.0)
			{
				map_mean += local_map->value[i] + global_map->carmen_map.map[global.x][global.y];
			}
		}
	}

	return (map_mean / (2.0 * local_map->number_of_known_points_on_the_map));
}


void
correlation_correction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_map)
{
	double map_mean = calc_map_mean(global_map, local_map, &filter->particles[0]);
	double min_weight = 0.00001;
	double w;
	
	for (int i = 0; i < filter->param->num_particles; i++)
	{
		w = pearson_correlation_correction(global_map, local_map, &(filter->particles[i]), map_mean);
		w = 1.0 - w > min_weight ? w : min_weight;
		filter->particles[i].weight = - w * w * 6;
	}
	convert_particles_log_odd_weights_to_prob(filter);
}


void
localize_map_correlation_correction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_t *localize_map,
		carmen_compact_map_t *local_map)
{
//	static int vezes = 0;

//	FILE *p = fopen("p.txt", "a");
//	fprintf(p, "vezes %d, number_of_known_points_on_the_map %d\n", vezes++, local_map->number_of_known_points_on_the_map);
//	fprintf(p, "ox %lf, oy %lf, sx %d, sy %d\n",
//			localize_map->config.x_origin, localize_map->config.y_origin, localize_map->config.x_size, localize_map->config.y_size);
	for (int i = 0; i < filter->param->num_particles; i++)
	{
//		if (vezes == 3)
//			vezes = 3;
		filter->particles[i].weight = map_particle_correction(localize_map, local_map, filter, i);
//		fprintf(p, "%d, w %lf,  x %lf, y %lf, theta %lf\n", i,
//				filter->particles[i].weight, filter->particles[i].x, filter->particles[i].y, filter->particles[i].theta);
	}
//	fprintf(p, "\n");
//	fclose(p);

	convert_particles_log_odd_weights_to_prob(filter);
}


void
localize_map_correlation_correction_with_outlier_rejection(carmen_localize_ackerman_particle_filter_p filter,
		carmen_localize_ackerman_map_t *localize_map, carmen_compact_map_t *local_map)
{
	compute_particles_weights_with_outlier_rejection(localize_map, local_map, filter);

	convert_particles_log_odd_weights_to_prob(filter);
}


//void
//localize_neural_correction(carmen_localize_ackerman_particle_filter_p filter,
//		carmen_localize_ackerman_map_t *localize_map, carmen_compact_map_t *local_map)
//{
//	if (generate_neural_ground_truth)
//	{
//		for (int i = 1; i < filter->param->num_particles; i++)
//		{
//			carmen_point_t delta_pose = get_delta_pose(filter->particles[i], filter->particles[0]);
//			save_delta_pose(delta_pose);
//			save_ofline_map(localize_map, );
//			save_online_map();
//		}
//	}
//}


static double
carmen_localize_ackerman_function_velodyne_evaluation(
		carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_p map,
		carmen_compact_map_t *local_map, carmen_compact_map_t *local_mean_remission_map,
		carmen_compact_map_t *local_variance_remission_map  __attribute__ ((unused)), int i)
{
	if (!filter->initialized)
		return 0.0;

	double w = 0.0;
	double min_weight = 0.00001;
	double map_mean;
	double inner_product_remission, norm_local_remission_map, norm_global_remission_map;
	double inner_product, norm_local_map, norm_global_map;

	switch (filter->param->correction_type)
	{
		case 0:
			w = map_particle_correction(map, local_map, filter, i);
			//w = exp(w);
			break;
		case 1:
			inner_product_between_maps(&inner_product, &norm_local_map, &norm_global_map, &map->carmen_map, local_map, &(filter->particles[i]));
			w = inner_product / (sqrt(norm_local_map) * sqrt(norm_global_map));
			w = 1.0 - (w > min_weight ? w : min_weight);
			w =exp(-w*w*100.0);
			break;
		case 2:
			w = cosine_correlation_correction_with_likelihood_map(map, local_map, &(filter->particles[i]));
			w = 1.0 - (w > min_weight ? w : min_weight);
			w =exp(-w*w*100.0);
			break;

		case 3:
			map_mean = calc_map_mean(map, local_map, &filter->particles[i]);
			w = pearson_correlation_correction(map, local_map, &(filter->particles[i]), map_mean);
			w = 1.0 - (w > min_weight ? w : min_weight);
			w =exp(-w*w*100.0);
			break;

		case 4:
			inner_product_between_maps(&inner_product, &norm_local_map, &norm_global_map, &map->carmen_mean_remission_map, local_mean_remission_map, &(filter->particles[i]));
			w = inner_product / (sqrt(norm_local_map) * sqrt(norm_global_map));
			w = 1.0 - (w > min_weight ? w : min_weight);
			w =exp(-w*w*100.0);
			break;

		case 5:
			inner_product_between_maps(&inner_product_remission, &norm_local_remission_map, &norm_global_remission_map, &map->carmen_mean_remission_map, local_mean_remission_map, &(filter->particles[i]));
			inner_product_between_maps(&inner_product, &norm_local_map, &norm_global_map, &map->carmen_map, local_map, &(filter->particles[i]));
			inner_product = inner_product + inner_product_remission;
			norm_local_map = norm_local_map + norm_local_remission_map;
			norm_global_map = norm_global_map + norm_global_remission_map;
			w = (inner_product) / (sqrt(norm_local_map) * sqrt(norm_global_map));
			w = 1.0 - (w > min_weight ? w : min_weight);
			w = exp(-w * w * 100.0);
			break;
	}

	return w;
}

#include <carmen/moving_objects_interface.h>

double
mahalanobis_distance(carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_mean_remission_map, carmen_localize_ackerman_particle_ipc_t *particle,
		int use_log_odds, double small_log_odds, double min_remission_variance)
{
	int i;
	cell_coords_t local, global;
	double sum = 0.0;
	double sin_theta, cos_theta;
	carmen_vector_2D_t robot_position;

	sin_theta = sin(particle->theta);
	cos_theta = cos(particle->theta);
	double map_center_x = (double) local_mean_remission_map->config.x_size * 0.5;
	double map_center_y = (double) local_mean_remission_map->config.y_size * 0.5;
	robot_position.x = particle->x - global_map->config.x_origin;
	robot_position.y = particle->y - global_map->config.y_origin;
	double robot_position_in_the_map_x = robot_position.x / local_mean_remission_map->config.resolution;
	double robot_position_in_the_map_y = robot_position.y / local_mean_remission_map->config.resolution;

//	static double first_time = 0.0;
//	static int first_in = 1;
//	if (first_in == 1)
//	{
//		first_time = carmen_get_time();
//		first_in = 2;
//	}
//	FILE *caco = NULL;
//	double time = carmen_get_time();
//	if ((time - first_time > 15.0) && (first_in == 2))
//	{
//		caco = fopen("caco10.txt", "w");
//		first_in = 3;
//	}

	for (i = 0; i < local_mean_remission_map->number_of_known_points_on_the_map; i++)
	{
		local.x = local_mean_remission_map->coord_x[i];
		local.y = local_mean_remission_map->coord_y[i];

		calc_global_cell_coordinate_fast(&global, local, map_center_x, map_center_y,
					robot_position_in_the_map_x, robot_position_in_the_map_y, sin_theta, cos_theta);

		if (global.x >= 0 && global.y >= 0 && global.x < global_map->config.x_size && global.y < global_map->config.y_size)
		{
			double cell_val = local_mean_remission_map->value[i];
			double mean_map_val = global_map->carmen_mean_remission_map.map[global.x][global.y];
			double variance = global_map->carmen_variance_remission_map.map[global.x][global.y];

			if (variance < min_remission_variance)
				variance = min_remission_variance;

			if (mean_map_val == -1.0)
			{
				sum += small_log_odds;
				continue;
			}
			if ((mean_map_val == 0.0) || (cell_val == 0.0))
				continue;

			double exponent = (cell_val - mean_map_val) * (cell_val - mean_map_val) / (2.0 * variance);
			if (use_log_odds)
			{
				double p = (1.0 / sqrt(2.0 * M_PI * variance)) * exp(-exponent);
				sum += log(p / (1.0 - p)); // nao esta funcionando pois p fica maior que 1.0 devido ao denominador acima
			}
			else
				sum += -(exponent);// + 0.5 * log(2.0 * M_PI * variance)); // log da probabilidade: https://www.wolframalpha.com/input/?i=log((1%2Fsqr(2*p*v))*exp(-((x-m)%5E2)%2F(2*v))

//			if ((caco != NULL) && (first_in >= 3) && (first_in < 10))
//			{
//				fprintf(caco, "%lf %lf %lf %lf %lf\n", cell_val, mean_map_val, variance, exp(-exponent), sum);
//				fflush(caco);
//			}
		}
		else
			sum += small_log_odds;
	}
//	if ((caco != NULL) && (first_in == 10))
//	{
//		fclose(caco);
//		caco = NULL;
//	}
//
//	if (first_in >= 3)
//		first_in++;

	return (sum);
}


void
localize_map_mahalanobis_correction_with_remission_map(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_mean_remission_map)
{
	int i;
	double small_log_odds;

	if (filter->param->use_log_odds)
		small_log_odds = log(filter->param->small_remission_likelihood / (1.0 - filter->param->small_remission_likelihood));
	else
		small_log_odds = log(filter->param->small_remission_likelihood);

	for (i = 0; i < filter->param->num_particles; i++)
		filter->particles[i].weight = filter->param->particles_normalize_factor * mahalanobis_distance(global_map, local_mean_remission_map, &filter->particles[i], filter->param->use_log_odds,
				small_log_odds, filter->param->min_remission_variance);

	convert_particles_log_odd_weights_to_prob(filter);
}


void
mahalanobis_distance_with_outlier_rejection(carmen_localize_ackerman_map_t *localize_map, carmen_compact_map_t *local_map, carmen_localize_ackerman_particle_filter_p filter)
{
	double map_center_x = (double) local_map->config.x_size * 0.5;
	double map_center_y = (double) local_map->config.y_size * 0.5;
	double small_log_weight;

	int use_log_odds = filter->param->use_log_odds;

	if (use_log_odds)
		small_log_weight = log(filter->param->tracking_beam_minlikelihood / (1.0 - filter->param->tracking_beam_minlikelihood));
	else
		small_log_weight = log(filter->param->tracking_beam_minlikelihood);

	int *count = NULL;
	int num_readings = local_map->number_of_known_points_on_the_map;
	if (num_readings > 0)
		count = (int *) calloc(num_readings, sizeof(int));

	double min_remission_variance = filter->param->min_remission_variance;
	double **mean_map = localize_map->carmen_mean_remission_map.map;
	double **variance_map = localize_map->carmen_variance_remission_map.map;
	double *local_map_value = local_map->value;
	double **temp_weights = filter->temp_weights;

	for (int i = 0; i < filter->param->num_particles; i++)
	{
		carmen_localize_ackerman_particle_ipc_t *particle = &(filter->particles[i]);

		double sin_theta = sin(particle->theta);
		double cos_theta = cos(particle->theta);
		double robot_position_in_the_map_x = (particle->x - localize_map->config.x_origin) / local_map->config.resolution;
		double robot_position_in_the_map_y = (particle->y - localize_map->config.y_origin) / local_map->config.resolution;

		particle->weight = 0.0;
		for (int laser_reading = 0; laser_reading < num_readings; laser_reading++)
		{
			cell_coords_t local_cell;
			local_cell.x = local_map->coord_x[laser_reading];
			local_cell.y = local_map->coord_y[laser_reading];

			cell_coords_t global_cell;
			calc_global_cell_coordinate_fast(&global_cell, local_cell, map_center_x, map_center_y,
						robot_position_in_the_map_x, robot_position_in_the_map_y, sin_theta, cos_theta);

			if (global_cell.x >= 0 && global_cell.y >= 0 && global_cell.x < localize_map->config.x_size && global_cell.y < localize_map->config.y_size)
			{
				double cell_val = local_map_value[laser_reading];
				double mean_map_val = mean_map[global_cell.x][global_cell.y];
				double variance = variance_map[global_cell.x][global_cell.y];

				if (variance < min_remission_variance)
					variance = min_remission_variance;

				if ((mean_map_val <= 0.0) || (cell_val == 0.0))
				{
					temp_weights[i][laser_reading] = small_log_weight;
				}
				else
				{
					double exponent = (cell_val - mean_map_val) * (cell_val - mean_map_val) /
							(filter->param->remission_variance_multiplier * variance);
					if (use_log_odds)
					{
						double p = (1.0 / sqrt(2.0 * M_PI * variance)) * exp(-exponent);

						// nao esta funcionando pois p fica maior que 1.0 devido ao denominador acima
						// pq o p fica maior que 1?
						temp_weights[i][laser_reading] = log(p / (1.0 - p));
					}
					else
						temp_weights[i][laser_reading] = -(exponent);// + 0.5 * log(2.0 * M_PI * variance)); // log da probabilidade: https://www.wolframalpha.com/input/?i=log((1%2Fsqr(2*p*v))*exp(-((x-m)%5E2)%2F(2*v))
				}
				if (temp_weights[i][laser_reading] <= small_log_weight)
					count[laser_reading] += 1;
			}
			else
			{
				temp_weights[i][laser_reading] = small_log_weight;
				count[laser_reading] += 1;
			}
		}
	}

//	int discarded = 0;
	for (int laser_reading = 0; laser_reading < num_readings; laser_reading++)
	{
		if (((double) count[laser_reading] / (double) filter->param->num_particles) < filter->param->outlier_fraction)
			for (int i = 0; i < filter->param->num_particles; i++)
				filter->particles[i].weight += temp_weights[i][laser_reading];
//		else
//			discarded++;
	}

//	printf("percentage discarded %lf, discarded %d, num_readings %d\n", (double) discarded / (double) num_readings, discarded, num_readings);
	for (int i = 0; i < filter->param->num_particles; i++)
		filter->particles[i].weight *= filter->param->particles_normalize_factor;

	if (num_readings > 0)
		free(count);
}


void
localize_map_mahalanobis_correction_with_remission_map_and_outlier_rejection(carmen_localize_ackerman_particle_filter_p filter,
		carmen_localize_ackerman_map_t *global_map, carmen_compact_map_t *local_mean_remission_map)
{
	mahalanobis_distance_with_outlier_rejection(global_map, local_mean_remission_map, filter);

	convert_particles_log_odd_weights_to_prob(filter);
}


void
carmen_localize_ackerman_velodyne_correction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_p localize_map,
		carmen_compact_map_t *local_map, carmen_compact_map_t *local_mean_remission_map,
		carmen_compact_map_t *local_variance_remission_map  __attribute__ ((unused)),
		carmen_localize_ackerman_binary_map_t *binary_map __attribute__ ((unused)))
{
	if (!filter->initialized)
		return;

	switch (filter->param->correction_type)
	{
		case 0:
			// The localize_map used in this function must be in log_odds and the local_map in probabilities
//			localize_map_correlation_correction(filter, localize_map, local_map);
			localize_map_correlation_correction_with_outlier_rejection(filter, localize_map, local_map);
// 			Para ver o mapa de referencia usado na localizacao no navigator_gui2, coloque flags = CARMEN_GRAPHICS_LOG_ODDS | CARMEN_GRAPHICS_INVERT
//			no case CARMEN_MOVING_OBJECTS_MAP_v da funcao GtkGui::navigator_graphics_set_flags() de gtk_gui.cpp,
//			descomente as 8 linhas abaixo, compile este codigo e o navigator_gui2. Use o mapa de Moving Objects no navigator_gui2. Nao se esquecca de reverter tudo ao original!
//			carmen_moving_objects_map_message moving_objects_map_message;
//			moving_objects_map_message.complete_map = localize_map->complete_prob;
//			moving_objects_map_message.size = localize_map->config.x_size * localize_map->config.y_size;
//			moving_objects_map_message.config = localize_map->config;
//			moving_objects_map_message.timestamp = carmen_get_time();
//			moving_objects_map_message.host = carmen_get_host();
//			carmen_moving_objects_map_publish_message(&moving_objects_map_message);

// 			Para ver o mapa instantaneo usado na localizacao no navigator_gui2, coloque flags = 0
//			no case CARMEN_MOVING_OBJECTS_MAP_v da funcao GtkGui::navigator_graphics_set_flags() de gtk_gui.cpp,
//			descomente as 13 linhas abaixo, compile este codigo e o navigator_gui2. Use o mapa de Moving Objects no navigator_gui2. Nao se esquecca de reverter tudo ao original!
//			carmen_moving_objects_map_message moving_objects_map_message;
//			carmen_map_t temp_map;
//			carmen_grid_mapping_create_new_map(&temp_map, local_map->config.x_size, local_map->config.y_size, local_map->config.resolution, 'm');
//			memset(temp_map.complete_map, 0, temp_map.config.x_size * temp_map.config.y_size * sizeof(double));
//			carmen_prob_models_uncompress_compact_map(&temp_map, local_map);
//			moving_objects_map_message.complete_map = temp_map.complete_map;
//			moving_objects_map_message.size = temp_map.config.x_size * temp_map.config.y_size;
//			moving_objects_map_message.config = temp_map.config;
//			moving_objects_map_message.timestamp = carmen_get_time();
//			moving_objects_map_message.host = carmen_get_host();
//			carmen_moving_objects_map_publish_message(&moving_objects_map_message);
//			free(temp_map.complete_map);
//			free(temp_map.map);

//			carmen_map_t temp_map;
//			temp_map.config = localize_map->config;
//			temp_map.complete_map = localize_map->complete_prob;
//			temp_map.map = localize_map->prob;
//			carmen_grid_mapping_save_map((char *) "test.map", &temp_map);
			break;

		case 1:
			cosine_correction(filter, localize_map, local_map);
			break;

		case 2:
			cosine_correction_with_likelihood_map(filter, localize_map, local_map);
			break;

		case 3:
			correlation_correction(filter, localize_map, local_map);
			break;

		case 4:
			cosine_correction_with_remission_map(filter, localize_map, local_mean_remission_map);

//			carmen_moving_objects_map_message moving_objects_map_message;
//			moving_objects_map_message.complete_map = localize_map->carmen_mean_remission_map.complete_map;
//			moving_objects_map_message.size = localize_map->config.x_size * localize_map->config.y_size;
//			moving_objects_map_message.config = localize_map->config;
//			moving_objects_map_message.timestamp = carmen_get_time();
//			moving_objects_map_message.host = carmen_get_host();
//			carmen_moving_objects_map_publish_message(&moving_objects_map_message);

//			carmen_moving_objects_map_message moving_objects_map_message;
//			carmen_map_t temp_map;
//			carmen_grid_mapping_create_new_map(&temp_map, local_map->config.x_size, local_map->config.y_size, local_map->config.resolution, 'm');
//			memset(temp_map.complete_map, 0, temp_map.config.x_size * temp_map.config.y_size * sizeof(double));
//			carmen_prob_models_uncompress_compact_map(&temp_map, local_mean_remission_map);
//			moving_objects_map_message.complete_map = temp_map.complete_map;
//			moving_objects_map_message.size = temp_map.config.x_size * temp_map.config.y_size;
//			moving_objects_map_message.config = temp_map.config;
//			moving_objects_map_message.timestamp = carmen_get_time();
//			moving_objects_map_message.host = carmen_get_host();
//			carmen_moving_objects_map_publish_message(&moving_objects_map_message);
//			free(temp_map.complete_map);
//			free(temp_map.map);
			break;

		case 5:
			cosine_correction_with_remission_map_and_grid_map(filter, localize_map, local_mean_remission_map, local_map);
			break;

		case 6:
			cosine_correction_with_remission_map_and_log_likelihood(filter, localize_map, local_mean_remission_map, local_map);
			break;

		case 7:
//			localize_map_mahalanobis_correction_with_remission_map(filter, localize_map, local_mean_remission_map);
			localize_map_mahalanobis_correction_with_remission_map_and_outlier_rejection(filter, localize_map, local_mean_remission_map);
//			carmen_moving_objects_map_message moving_objects_map_message;
//			moving_objects_map_message.size = localize_map->config.x_size * localize_map->config.y_size;
//			double *new_map_x = (double *) malloc(moving_objects_map_message.size * sizeof(double));
//			moving_objects_map_message.complete_map = new_map_x;
//			for (int i = 0; i < moving_objects_map_message.size; i++)
//			{
////				if (localize_map->carmen_mean_remission_map.complete_map[i] >= 0.0)
//					moving_objects_map_message.complete_map[i] = localize_map->carmen_mean_remission_map.complete_map[i];
////				else
////					moving_objects_map_message.complete_map[i] = -1.0;
//			}
//			moving_objects_map_message.config = localize_map->config;
//			moving_objects_map_message.timestamp = carmen_get_time();
//			moving_objects_map_message.host = carmen_get_host();
//			carmen_moving_objects_map_publish_message(&moving_objects_map_message);
//			free(new_map_x);
//
//			carmen_moving_objects_map_message moving_objects_map_message;
//			carmen_map_t temp_map;
//			carmen_grid_mapping_create_new_map(&temp_map, local_mean_remission_map->config.x_size, local_mean_remission_map->config.y_size, local_mean_remission_map->config.resolution, 'c');
//			carmen_prob_models_uncompress_compact_map(&temp_map, local_mean_remission_map);
//			moving_objects_map_message.complete_map = temp_map.complete_map;
//			moving_objects_map_message.size = temp_map.config.x_size * temp_map.config.y_size;
//			moving_objects_map_message.config = temp_map.config;
//			moving_objects_map_message.timestamp = carmen_get_time();
//			moving_objects_map_message.host = carmen_get_host();
//			carmen_moving_objects_map_publish_message(&moving_objects_map_message);
//			free(temp_map.complete_map);
//			free(temp_map.map);
			break;

//		case 8:
//					// The localize_map used in this function must be in log_odds and the local_map in probabilities
//		//			localize_map_correlation_correction(filter, localize_map, local_map);
//					localize_neural_correction(filter, localize_map, local_map);
//					// para ver este mapa no navigator_gui2 coloque CARMEN_GRAPHICS_LOG_ODDS | CARMEN_GRAPHICS_INVERT na linha 930 de gtk_gui.cpp
//		//			carmen_moving_objects_map_message moving_objects_map_message;
//		//			moving_objects_map_message.complete_map = localize_map->complete_prob;
//		//			moving_objects_map_message.size = localize_map->config.x_size * localize_map->config.y_size;
//		//			moving_objects_map_message.config = localize_map->config;
//		//			moving_objects_map_message.timestamp = carmen_get_time();
//		//			moving_objects_map_message.host = carmen_get_host();
//		//			carmen_moving_objects_map_publish_message(&moving_objects_map_message);
//
//		//			carmen_moving_objects_map_message moving_objects_map_message;
//		//			carmen_map_t temp_map;
//		//			carmen_grid_mapping_create_new_map(&temp_map, local_map->config.x_size, local_map->config.y_size, local_map->config.resolution, 'm');
//		//			memset(temp_map.complete_map, 0, temp_map.config.x_size * temp_map.config.y_size * sizeof(double));
//		//			carmen_prob_models_uncompress_compact_map(&temp_map, local_map);
//		//			moving_objects_map_message.complete_map = temp_map.complete_map;
//		//			moving_objects_map_message.size = temp_map.config.x_size * temp_map.config.y_size;
//		//			moving_objects_map_message.config = temp_map.config;
//		//			moving_objects_map_message.timestamp = carmen_get_time();
//		//			moving_objects_map_message.host = carmen_get_host();
//		//			carmen_moving_objects_map_publish_message(&moving_objects_map_message);
//		//			free(temp_map.complete_map);
//		//			free(temp_map.map);
//
//		//			carmen_map_t temp_map;
//		//			temp_map.config = localize_map->config;
//		//			temp_map.complete_map = localize_map->complete_prob;
//		//			temp_map.map = localize_map->prob;
//		//			carmen_grid_mapping_save_map((char *) "test.map", &temp_map);
//					break;
	}
}


double
calc_new_particle_velocity(double pose, double particle_velocity, double pbest, double gbest, double max_velocity)
{
	double r1 = carmen_double_random(1.0);
	double r2 = carmen_double_random(1.0);

	double v = 0.729 * (particle_velocity + 2.05 * r1 * (pbest - pose) + 2.05 * r2 * (gbest - pose));

	v = v > max_velocity ? max_velocity : v;
	v = v < -max_velocity ? -max_velocity : v;

	return v;
}


void
update_swarm_particle_position(double *pose, double *particle_velocity, double reference_pose, double pbest, double gbest, double max_velocity, double max_displacement)
{
	double v = *particle_velocity;
	double p = *pose;


	v = calc_new_particle_velocity(p, v, pbest, gbest, max_velocity);
	p += v;

	if ((p - reference_pose) > max_displacement)
		p = reference_pose + max_displacement;
	if ((p - reference_pose) < -max_displacement)
		p = reference_pose - max_displacement;

	*particle_velocity = v;
	*pose = p;
}


void
update_swarm_particle_orientation(double *orientation, double *particle_angular_velocity, double reference_orientation, double pbest, double gbest, double max_angular_velocity, double max_angular_displacement)
{
	double v = *particle_angular_velocity;
	double o = *orientation;

	v = calc_new_particle_velocity(o, v, pbest, gbest, max_angular_velocity);
	o = carmen_normalize_theta(o + v);

	if((o - reference_orientation) > max_angular_displacement)
		o = reference_orientation + max_angular_displacement;
	if((o - reference_orientation) < -max_angular_displacement)
		o = reference_orientation - max_angular_displacement;

	*particle_angular_velocity = v;
	*orientation = o;
}


void
update_gbest_paricle(int particle_index, carmen_localize_ackerman_particle_filter_p filter, carmen_fused_odometry_message *fused_odometry)
{
	update_swarm_particle_position(&filter->particles[particle_index].x, &filter->swarm_velocity[particle_index].x, fused_odometry->pose.position.x,
			filter->swarm_pbest[particle_index].x, filter->swarm_gbest.x, filter->param->swarm_max_particle_velocity, filter->param->max_particle_displacement);

	update_swarm_particle_position(&filter->particles[particle_index].y, &filter->swarm_velocity[particle_index].y, fused_odometry->pose.position.y,
			filter->swarm_pbest[particle_index].y, filter->swarm_gbest.y, filter->param->swarm_max_particle_velocity, filter->param->max_particle_displacement);

	update_swarm_particle_orientation(&filter->particles[particle_index].theta, &filter->swarm_velocity[particle_index].theta, fused_odometry->pose.orientation.yaw,
			filter->swarm_pbest[particle_index].theta, filter->swarm_gbest.theta, filter->param->swarm_max_particle_angular_velocity, filter->param->max_particle_angular_displacement);
}


void
calc_new_gbest_swarm_paticles_position(carmen_localize_ackerman_particle_filter_p filter, carmen_fused_odometry_message *fused_odometry,carmen_localize_ackerman_map_p map,
		carmen_compact_map_t *local_map, carmen_compact_map_t *local_mean_remission_map,
		carmen_compact_map_t *local_variance_remission_map  __attribute__ ((unused)))
{
	int i;

	for (i = 0; i < filter->param->num_particles; i++)
	{
		update_gbest_paricle(i, filter, fused_odometry);

		filter->particles[i].weight = carmen_localize_ackerman_function_velodyne_evaluation(filter, map, local_map,
					local_mean_remission_map, local_variance_remission_map, i);

		if (filter->particles[i].weight > filter->swarm_pbest[i].weight)
		{
			filter->swarm_pbest[i] = filter->particles[i];
			if (filter->swarm_pbest[i].weight > filter->swarm_gbest.weight)
				filter->swarm_gbest = filter->swarm_pbest[i];
		}
	}
}


void
update_lbest_paricle(int particle_index, int lparticle_index, int rparticle_index, carmen_localize_ackerman_particle_filter_p filter, carmen_fused_odometry_message *fused_odometry)
{
	if (filter->swarm_pbest[lparticle_index].weight > filter->swarm_pbest[rparticle_index].weight)
		filter->swarm_gbest = filter->swarm_pbest[lparticle_index];
	else
		filter->swarm_gbest = filter->swarm_pbest[rparticle_index];

	update_swarm_particle_position(&filter->particles[particle_index].x, &filter->swarm_velocity[particle_index].x, fused_odometry->pose.position.x,
			filter->swarm_pbest[particle_index].x, filter->swarm_gbest.x, filter->param->swarm_max_particle_velocity, filter->param->max_particle_displacement);

	update_swarm_particle_position(&filter->particles[particle_index].y, &filter->swarm_velocity[particle_index].y, fused_odometry->pose.position.y,
			filter->swarm_pbest[particle_index].y, filter->swarm_gbest.y, filter->param->swarm_max_particle_velocity, filter->param->max_particle_displacement);

	update_swarm_particle_orientation(&filter->particles[particle_index].theta, &filter->swarm_velocity[particle_index].theta, fused_odometry->pose.orientation.yaw,
			filter->swarm_pbest[particle_index].theta, filter->swarm_gbest.theta, filter->param->swarm_max_particle_angular_velocity, filter->param->max_particle_angular_displacement);
}


void
calc_new_lbest_swarm_paticles_position(carmen_localize_ackerman_particle_filter_p filter, carmen_fused_odometry_message *fused_odometry)
{
	int particle_index, lparticle_index, rparticle_index;

	lparticle_index = filter->param->num_particles - 1;
	rparticle_index = 1;

	for (particle_index = 0; particle_index < filter->param->num_particles - 1; particle_index++)
	{
		update_lbest_paricle(particle_index, lparticle_index, rparticle_index, filter, fused_odometry);
		lparticle_index = particle_index;
		rparticle_index = particle_index + 2;
	}

	lparticle_index = filter->param->num_particles - 2;
	rparticle_index = 0;

	update_lbest_paricle(particle_index, lparticle_index, rparticle_index, filter, fused_odometry);
}


void
swarm_get_best_particle(carmen_localize_ackerman_particle_filter_p filter)
{
	int j;

	for (j = 0; j < filter->param->num_particles; j++)
	{
		if (filter->swarm_pbest[j].weight < filter->particles[j].weight)
		{
			filter->swarm_pbest[j] = filter->particles[j];
			if (filter->swarm_gbest.weight < filter->particles[j].weight)
				filter->swarm_gbest = filter->particles[j];
		}
	}
}


void
swarm_initialize(carmen_localize_ackerman_particle_filter_p filter)
{
	memset( filter->swarm_pbest, 	0, filter->param->num_particles * sizeof(carmen_localize_ackerman_particle_ipc_t));
	memset( filter->swarm_velocity, 0, filter->param->num_particles * sizeof(carmen_localize_ackerman_particle_ipc_t));
	memset(&filter->swarm_gbest, 	0, sizeof(carmen_localize_ackerman_particle_ipc_t));
}


void
swarm(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_p map,
		carmen_compact_map_t *local_map, carmen_compact_map_t *local_mean_remission_map,
		carmen_compact_map_t *local_variance_remission_map, carmen_fused_odometry_message *fused_odometry, carmen_localize_ackerman_binary_map_t *binary_map)
{
	int i = 0;

	swarm_initialize(filter);

	carmen_localize_ackerman_velodyne_correction(filter, map, local_map,
								local_mean_remission_map, local_variance_remission_map, binary_map);
	swarm_get_best_particle(filter);

	for (i = 0; i < filter->param->swarm_num_iteration; i++)
	{
		calc_new_lbest_swarm_paticles_position(filter, fused_odometry);// map, local_map,
//				local_mean_remission_map, local_variance_remission_map);
		carmen_localize_ackerman_velodyne_correction(filter, map, local_map,
						local_mean_remission_map, local_variance_remission_map, binary_map);
		swarm_get_best_particle(filter);
	}
	memcpy( filter->particles, 	filter->swarm_pbest, filter->param->num_particles * sizeof(carmen_localize_ackerman_particle_ipc_t));
}


void
update_de_particle_position(double *pose, double reference_pose, double pbest_a, double pbest_b, double pbest_c, double CR, double F, double max_displacement)
{
	double p = *pose;

	if (carmen_double_random(1.0) < CR)
	{
		p = pbest_a + F * (pbest_b - pbest_c);

		if ((p - reference_pose) > max_displacement)
			p = reference_pose + max_displacement;
		if ((p - reference_pose) < -max_displacement)
			p = reference_pose - max_displacement;
	}

	*pose = p;
}


void
update_de_particle_orientation(double *orientation, double reference_orientation, double pbest_a, double pbest_b, double pbest_c, double CR, double F, double max_angular_displacement)
{
	double o = *orientation;

	if (carmen_double_random(1.0) < CR)
	{
		o = carmen_normalize_theta(pbest_a + F * (pbest_b - pbest_c));

		if ((o - reference_orientation) > max_angular_displacement)
			o = reference_orientation + max_angular_displacement;
		if ((o - reference_orientation) < -max_angular_displacement)
			o = reference_orientation - max_angular_displacement;
	}

	*orientation = carmen_normalize_theta(o);
}


void
update_hade_particle_position(double *pose, double reference_pose, double pbest_a, double pbest_b, double pbest_c, double CR, double F, double lambda, double max_displacement)
{
	double p = *pose;

	if (carmen_double_random(1.0) < CR)
	{
		p = p + F * (p - pbest_a) + lambda * (pbest_b - pbest_c);

		if ((p - reference_pose) > max_displacement)
			p = reference_pose + max_displacement;
		if ((p - reference_pose) < -max_displacement)
			p = reference_pose - max_displacement;
	}

	*pose = p;
}


void
update_hade_particle_orientation(double *orientation, double reference_orientation, double pbest_a, double pbest_b, double pbest_c, double CR, double F, double lambda, double max_angular_displacement)
{
	double o = *orientation;

	if (carmen_double_random(1.0) < CR)
	{
		o = carmen_normalize_theta(o + F * (o - pbest_a) + lambda * (pbest_b - pbest_c));

		if ((o - reference_orientation) > max_angular_displacement)
			o = reference_orientation + max_angular_displacement;
		if ((o - reference_orientation) < -max_angular_displacement)
			o = reference_orientation - max_angular_displacement;
	}

	*orientation = carmen_normalize_theta(o);
}



void
calc_new_de_particle(carmen_localize_ackerman_particle_filter_p filter, carmen_fused_odometry_message *fused_odometry, carmen_localize_ackerman_map_p map,
		carmen_compact_map_t *local_map, carmen_compact_map_t *local_mean_remission_map,
		carmen_compact_map_t *local_variance_remission_map  __attribute__ ((unused)))
{
	int i, a, b, c;


	for (i = 0; i < filter->param->num_particles; i++)
	{
		do {a = (int)carmen_uniform_random(0.0, filter->param->num_particles);} while(a == i);
		do {b = (int)carmen_uniform_random(0.0, filter->param->num_particles);} while(b == i || b == a);
		do {c = (int)carmen_uniform_random(0.0, filter->param->num_particles);} while(c == i || c == a || b == a);

		filter->particles[i] = filter->swarm_pbest[i];

		update_de_particle_position(&filter->particles[i].x, fused_odometry->pose.position.x, filter->swarm_gbest.x, filter->swarm_pbest[b].x, filter->swarm_pbest[c].x, filter->param->de_crossover_rate, filter->param->de_mutation_rate, filter->param->max_particle_displacement);
		update_de_particle_position(&filter->particles[i].y, fused_odometry->pose.position.y, filter->swarm_gbest.y, filter->swarm_pbest[b].y, filter->swarm_pbest[c].y, filter->param->de_crossover_rate, filter->param->de_mutation_rate, filter->param->max_particle_displacement);
		update_de_particle_orientation(&filter->particles[i].theta, fused_odometry->pose.orientation.yaw, filter->swarm_gbest.theta, filter->swarm_pbest[b].theta, filter->swarm_pbest[c].theta,  filter->param->de_crossover_rate, filter->param->de_mutation_rate, filter->param->max_particle_angular_displacement);

		filter->particles[i].weight = carmen_localize_ackerman_function_velodyne_evaluation(filter, map, local_map,
					local_mean_remission_map, local_variance_remission_map, i);

		if (filter->particles[i].weight > filter->swarm_pbest[i].weight)
		{
			filter->swarm_pbest[i] = filter->particles[i];
			if (filter->swarm_pbest[i].weight > filter->swarm_gbest.weight)
			{
				filter->swarm_gbest = filter->swarm_pbest[i];
			}
		}
	}
}


void
calc_new_hade_particle(carmen_localize_ackerman_particle_filter_p filter, carmen_fused_odometry_message *fused_odometry, carmen_localize_ackerman_map_p map,
		carmen_compact_map_t *local_map, carmen_compact_map_t *local_mean_remission_map,
		carmen_compact_map_t *local_variance_remission_map  __attribute__ ((unused)))
{
	int i, a, b, c;


	for (i = 0; i < filter->param->num_particles; i++)
	{
		do {a = (int)carmen_uniform_random(0.0, filter->param->num_particles);} while(a == i);
		do {b = (int)carmen_uniform_random(0.0, filter->param->num_particles);} while(b == i || b == a);
		do {c = (int)carmen_uniform_random(0.0, filter->param->num_particles);} while(c == i || c == a || b == a);

		filter->particles[i] = filter->swarm_pbest[i];
		double lambda = fabs(filter->swarm_pbest[b].weight - filter->swarm_pbest[c].weight);
		double F = fabs(filter->swarm_gbest.weight - filter->swarm_pbest[i].weight);

		update_hade_particle_position(&filter->particles[i].x, fused_odometry->pose.position.x, filter->swarm_pbest[a].x, filter->swarm_pbest[b].x, filter->swarm_pbest[c].x, filter->param->de_crossover_rate, F, lambda, filter->param->max_particle_displacement);
		update_hade_particle_position(&filter->particles[i].y, fused_odometry->pose.position.y, filter->swarm_pbest[a].y, filter->swarm_pbest[b].y, filter->swarm_pbest[c].y, filter->param->de_crossover_rate, F, lambda, filter->param->max_particle_displacement);
		update_hade_particle_orientation(&filter->particles[i].theta, fused_odometry->pose.orientation.yaw, filter->swarm_pbest[a].theta, filter->swarm_pbest[b].theta, filter->swarm_pbest[c].theta,  filter->param->de_crossover_rate, F, lambda, filter->param->max_particle_angular_displacement);

		filter->particles[i].weight = carmen_localize_ackerman_function_velodyne_evaluation(filter, map, local_map,
					local_mean_remission_map, local_variance_remission_map, i);

		if (filter->particles[i].weight > filter->swarm_pbest[i].weight)
		{
			filter->swarm_pbest[i] = filter->particles[i];
			if (filter->swarm_pbest[i].weight > filter->swarm_gbest.weight)
			{
				filter->swarm_gbest = filter->swarm_pbest[i];
			}
		}
	}
}


void
de(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_p map,
		carmen_compact_map_t *local_map, carmen_compact_map_t *local_mean_remission_map,
		carmen_compact_map_t *local_variance_remission_map, carmen_fused_odometry_message *fused_odometry, carmen_localize_ackerman_binary_map_t *binary_map)
{
	int i = 0;

	swarm_initialize(filter);

	carmen_localize_ackerman_velodyne_correction(filter, map, local_map,
			local_mean_remission_map, local_variance_remission_map, binary_map);
	swarm_get_best_particle(filter);

	for (i = 0; i < filter->param->de_num_iteration; i++)
	{
		calc_new_de_particle(filter, fused_odometry, map, local_map,
				local_mean_remission_map, local_variance_remission_map);
		carmen_localize_ackerman_velodyne_correction(filter, map, local_map,
				local_mean_remission_map, local_variance_remission_map, binary_map);
		swarm_get_best_particle(filter);
	}

	memcpy( filter->particles, 	filter->swarm_pbest, filter->param->num_particles * sizeof(carmen_localize_ackerman_particle_ipc_t));
}


void
hade(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_p map,
		carmen_compact_map_t *local_map, carmen_compact_map_t *local_mean_remission_map,
		carmen_compact_map_t *local_variance_remission_map, carmen_fused_odometry_message *fused_odometry, carmen_localize_ackerman_binary_map_t *binary_map)
{
	int i = 0;

	swarm_initialize(filter);

	carmen_localize_ackerman_velodyne_correction(filter, map, local_map,
			local_mean_remission_map, local_variance_remission_map, binary_map);
	swarm_get_best_particle(filter);

	for (i = 0; i < filter->param->de_num_iteration; i++)
	{
		calc_new_de_particle(filter, fused_odometry, map, local_map,
				local_mean_remission_map, local_variance_remission_map);
	}

	memcpy( filter->particles, 	filter->swarm_pbest, filter->param->num_particles * sizeof(carmen_localize_ackerman_particle_ipc_t));
}


void
carmen_localize_ackerman_velodyne_prediction(carmen_localize_ackerman_particle_filter_p filter, carmen_base_ackerman_odometry_message *odometry,
		carmen_xsens_global_quat_message *xsens_global_quat_message,
		double velodyne_timestamp, double distance_between_front_and_rear_axles)
{
	if (!filter->initialized)
		return;

	if (filter->param->prediction_type == 2)
		carmen_localize_ackerman_incorporate_IMU(filter, xsens_global_quat_message, distance_between_front_and_rear_axles, velodyne_timestamp - filter->last_timestamp);
	else if (filter->param->prediction_type == 1)
		carmen_localize_ackerman_incorporate_velocity_odometry(filter, odometry->v, odometry->phi, distance_between_front_and_rear_axles, velodyne_timestamp - filter->last_timestamp);
	else // differential prediction (v and w)
		carmen_localize_ackerman_incorporate_odometry(filter, odometry->v, odometry->phi, distance_between_front_and_rear_axles, velodyne_timestamp - filter->last_timestamp);

	filter->last_timestamp = velodyne_timestamp;
}


void
carmen_localize_ackerman_velodyne_resample(carmen_localize_ackerman_particle_filter_p filter)
{
	/* check if it is time to resample */
	if (filter->distance_travelled >= filter->param->update_distance)
	{
		velodyne_resample(filter);
		filter->distance_travelled = 0.0;
		filter->initialized = 1;
	}
}


void
carmen_localize_ackerman_laser_scan_gd(int num_readings, double *range, double angular_resolution, double first_beam_angle,
		carmen_point_p laser_pos, double forward_offset, carmen_localize_ackerman_map_p map, int laser_skip)
{
	double grad_x, grad_y, grad_theta, range_x, range_y, theta;
	int x_l, y_l, count = 0, i;

	double angular_res_in_degrees = carmen_radians_to_degrees(angular_resolution);

	do {
		grad_x = 0;
		grad_y = 0;
		grad_theta = 0;
		for(i = 0; i < num_readings; i += laser_skip) {

			theta = laser_pos->theta + first_beam_angle + i * angular_resolution;

			range_x = range[i] * cos(theta);
			range_y = range[i] * sin(theta);
			x_l = (int)(((laser_pos->x - map->config.x_origin) + forward_offset * cos(laser_pos->theta) +
					range_x) / map->config.resolution);
			y_l = (int)(((laser_pos->y - map->config.y_origin) + forward_offset * sin(laser_pos->theta) +
					range_y) / map->config.resolution);

			if(x_l >= 0 && y_l >= 0 && x_l < map->config.x_size &&
					y_l < map->config.y_size) {
				grad_x += map->x_offset[x_l][y_l];
				grad_y += map->y_offset[x_l][y_l];
				grad_theta += range_x * map->y_offset[x_l][y_l] -
						range_y * map->x_offset[x_l][y_l];
			}
		}

		/** what is the meaning of this? should this be adapted according to the fov ?*/
		/*     grad_x *= K_T * 180.0 / num_readings; */
		/*     grad_y *= K_T * 180.0 / num_readings; */
		/*     grad_theta *= K_ROT * 180.0 / num_readings; */

		grad_x *= K_T * angular_res_in_degrees;
		grad_y *= K_T * angular_res_in_degrees;
		grad_theta *= K_ROT * angular_res_in_degrees;

		laser_pos->x += grad_x;
		laser_pos->y += grad_y;
		laser_pos->theta += grad_theta;
		count++;
	} while(count < 20 && (grad_x > 0.05 || grad_y < 0.05 ||
			grad_theta < 0.25 * 180.0 / M_PI));
}


void
carmen_localize_ackerman_summarize_swarm(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary)
{
	double mean_x, mean_y, mean_theta_x, mean_theta_y;
	double diff_x, diff_y, diff_theta, std_x, std_y, std_theta, xy_cov;
	double total_weight = 0;
	int i;

	summary->converged = 1;//!filter->converged;

	/* compute mean particle pose */
	mean_x = 0;
	mean_y = 0;
	mean_theta_x = 0;
	mean_theta_y = 0;
	for(i = 0; i < filter->param->num_particles; i++)
	{
		mean_x += filter->particles[i].x * filter->particles[i].weight;
		mean_y += filter->particles[i].y * filter->particles[i].weight;
		mean_theta_x += cos(filter->particles[i].theta) * filter->particles[i].weight;
		mean_theta_y += sin(filter->particles[i].theta) * filter->particles[i].weight;

		total_weight += filter->particles[i].weight;
	}
	summary->mean.x = mean_x / total_weight;
	summary->mean.y = mean_y / total_weight;
	summary->mean.theta = atan2(mean_theta_y, mean_theta_x);
	summary->odometry_pos = filter->last_odometry_position;

	/* compute std particle pose */
	std_x = 0;
	std_y = 0;
	std_theta = 0;
	xy_cov = 0;
	for(i = 0; i < filter->param->num_particles; i++) {
		diff_x = (filter->particles[i].x - summary->mean.x);
		diff_y = (filter->particles[i].y - summary->mean.y);
		diff_theta = carmen_normalize_theta(filter->particles[i].theta -
				summary->mean.theta);
		std_x += carmen_square(diff_x);
		std_y += carmen_square(diff_y);
		std_theta += carmen_square(diff_theta);
		xy_cov += diff_x * diff_y;
	}
	summary->std.x = sqrt(std_x / filter->param->num_particles);
	summary->std.y = sqrt(std_y / filter->param->num_particles);
	summary->std.theta = sqrt(std_theta / filter->param->num_particles);
	summary->xy_cov = sqrt(xy_cov / filter->param->num_particles);

	summary->mean.x = filter->swarm_gbest.x;
	summary->mean.y = filter->swarm_gbest.y;
	summary->mean.theta = filter->swarm_gbest.theta;
}


void 
carmen_localize_ackerman_summarize_velodyne(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary)
{
	double mean_x, mean_y, mean_theta_x, mean_theta_y, mean_v, mean_phi, mean_phi_bias;
	double diff_x, diff_y, diff_theta, std_x, std_y, std_theta, xy_cov;	
	double total_weight = 0;
	int i;

	summary->converged = 1;//!filter->converged;

	/* compute mean particle pose */
	mean_x = 0.0;
	mean_y = 0.0;
	mean_theta_x = 0.0;
	mean_theta_y = 0.0;
	mean_phi_bias = 0.0;
	mean_v = 0.0;
	mean_phi = 0.0;
	for (i = 0; i < filter->param->num_particles; i++)
	{
		mean_x += filter->particles[i].x * filter->particles[i].weight;
		mean_y += filter->particles[i].y * filter->particles[i].weight;
		mean_theta_x += cos(filter->particles[i].theta) * filter->particles[i].weight;
		mean_theta_y += sin(filter->particles[i].theta) * filter->particles[i].weight;
		mean_v += filter->particles[i].v * filter->particles[i].weight;
		mean_phi += filter->particles[i].phi * filter->particles[i].weight;
		mean_phi_bias += filter->particles[i].phi_bias * filter->particles[i].weight;
		
		total_weight += filter->particles[i].weight;
	}
	summary->mean.x = mean_x / total_weight;
	summary->mean.y = mean_y / total_weight;
	summary->mean.theta = atan2(mean_theta_y, mean_theta_x);
	mean_v = mean_v / total_weight;
	mean_phi = mean_phi / total_weight;
	mean_phi_bias = mean_phi_bias / total_weight;

	summary->odometry_pos = filter->last_odometry_position;

	/* compute std particle pose */
	std_x = 0;
	std_y = 0;
	std_theta = 0;
	xy_cov = 0;
	for(i = 0; i < filter->param->num_particles; i++)
	{
		diff_x = (filter->particles[i].x - summary->mean.x);
		diff_y = (filter->particles[i].y - summary->mean.y);
		diff_theta = carmen_normalize_theta(filter->particles[i].theta -
				summary->mean.theta);
		std_x += carmen_square(diff_x);
		std_y += carmen_square(diff_y);
		std_theta += carmen_square(diff_theta);
		xy_cov += diff_x * diff_y;
	}
	summary->std.x = sqrt(std_x / filter->param->num_particles);
	summary->std.y = sqrt(std_y / filter->param->num_particles);
	summary->std.theta = sqrt(std_theta / filter->param->num_particles);

	summary->xy_cov = sqrt(xy_cov / filter->param->num_particles);
	// @@@ Alberto: temporario para teste dos beneficios de se usar mean_phi_bias no pipe de controle
//	summary->xy_cov = mean_phi_bias;

	// Add mean particle to the pool
	filter->particles[0].x = summary->mean.x;
	filter->particles[0].y = summary->mean.y;
	filter->particles[0].theta = summary->mean.theta;
	filter->particles[0].v = mean_v;
	filter->particles[0].phi = mean_phi;
	filter->particles[0].phi_bias = mean_phi_bias;
//	printf("mean_phi_bias %lf\n", mean_phi_bias);
}


void
carmen_localize_ackerman_summarize(carmen_localize_ackerman_particle_filter_p filter,
		carmen_localize_ackerman_summary_p summary,
		carmen_localize_ackerman_map_p map,
		int num_readings, double *range,
		double angular_resolution,
		double first_beam_angle,
		double forward_offset,
		int backwards)
{
	double mean_x, mean_y, mean_theta_x, mean_theta_y, angle;
	double diff_x, diff_y, diff_theta, std_x, std_y, std_theta, xy_cov;
	double *weights, max_weight = filter->particles[0].weight;
	double total_weight = 0;
	int i, x, y;

	summary->converged = 1;//!filter->converged;

	weights = (double *)calloc(filter->param->num_particles, sizeof(double));
	carmen_test_alloc(weights);
	for(i = 0; i < filter->param->num_particles; i++)
		if(filter->particles[i].weight > max_weight)
			max_weight = filter->particles[i].weight;
	for(i = 0; i < filter->param->num_particles; i++) {
		weights[i] = exp(filter->particles[i].weight - max_weight);
		total_weight += weights[i];
	}

	/* compute mean particle pose */
	mean_x = 0;
	mean_y = 0;
	mean_theta_x = 0;
	mean_theta_y = 0;
	for(i = 0; i < filter->param->num_particles; i++) {
		mean_x += filter->particles[i].x * weights[i];
		mean_y += filter->particles[i].y * weights[i];
		mean_theta_x += cos(filter->particles[i].theta) * weights[i];
		mean_theta_y += sin(filter->particles[i].theta) * weights[i];
	}
	summary->mean.x = mean_x / total_weight;
	summary->mean.y = mean_y / total_weight;
	if(mean_theta_x == 0)
		summary->mean.theta = 0;
	else
		summary->mean.theta = atan2(mean_theta_y, mean_theta_x);
	summary->odometry_pos = filter->last_odometry_position;

	/* compute std particle pose */
	std_x = 0;
	std_y = 0;
	std_theta = 0;
	xy_cov = 0;
	for(i = 0; i < filter->param->num_particles; i++) {
		diff_x = (filter->particles[i].x - summary->mean.x);
		diff_y = (filter->particles[i].y - summary->mean.y);
		diff_theta = carmen_normalize_theta(filter->particles[i].theta -
				summary->mean.theta);
		std_x += carmen_square(diff_x);
		std_y += carmen_square(diff_y);
		std_theta += carmen_square(diff_theta);
		xy_cov += diff_x * diff_y;
	}
	summary->std.x = sqrt(std_x / filter->param->num_particles);
	summary->std.y = sqrt(std_y / filter->param->num_particles);
	summary->std.theta = sqrt(std_theta / filter->param->num_particles);
	summary->xy_cov = sqrt(xy_cov / filter->param->num_particles);

	if(filter->param->do_scanmatching)
		carmen_localize_ackerman_laser_scan_gd(summary->num_readings,
				range,
				angular_resolution,
				first_beam_angle,
				&summary->mean,
				forward_offset, map, 1);

	/* compute mean scan */
	summary->num_readings = num_readings;
	for(i = 0; i < num_readings; i++) {
		summary->mean_scan[i].range = range[i];
		summary->mean_scan[i].mask = filter->laser_mask[i];
		if(backwards) {
			angle = summary->mean.theta + M_PI +
					first_beam_angle + i * angular_resolution;
			summary->mean_scan[i].x = summary->mean.x - forward_offset *
					cos(summary->mean.theta) + cos(angle) * range[i];
			summary->mean_scan[i].y = summary->mean.y - forward_offset *
					sin(summary->mean.theta) + sin(angle) * range[i];
		}
		else {
			angle = summary->mean.theta +
					first_beam_angle + i * angular_resolution;
			summary->mean_scan[i].x = summary->mean.x + forward_offset *
					cos(summary->mean.theta) + cos(angle) * range[i];
			summary->mean_scan[i].y = summary->mean.y + forward_offset *
					sin(summary->mean.theta) + sin(angle) * range[i];
		}
		x = ((summary->mean_scan[i].x - map->config.x_origin) / map->config.resolution);
		y = ((summary->mean_scan[i].y - map->config.y_origin) / map->config.resolution);
		if(x < 0 || y < 0 || x >= map->config.x_size || y >= map->config.y_size ||
				map->carmen_map.map[x][y] == -1)
			summary->mean_scan[i].prob = filter->param->tracking_beam_minlikelihood; //SMALL_PROB;
		else
			summary->mean_scan[i].prob = exp(map->prob[x][y]);
	}

	free(weights);
}


static int *
generates_ray_order(int size)
{
	int i;

	int *ray_order = (int *) malloc(size * sizeof(int));
	carmen_test_alloc(ray_order);

	for (i = 0; i < size; i++)
		ray_order[i] = i;

	return ray_order;
}


static void
sort_ray_order_by_vertical_correction_angles(sensor_parameters_t params)
{
	int aux = 0;

	for (int i = params.vertical_resolution - 1; i > 0; i--)
	{
		for (int j = 0; j < i; j++)
		{
			if (params.vertical_correction[params.ray_order[j]] > params.vertical_correction[params.ray_order[j + 1]])
			{
				aux = params.ray_order[j];
				params.ray_order[j] = params.ray_order[j + 1];
				params.ray_order[j + 1] = aux;
			}
		}
	}
}


static void
sort_vertical_correction_angles(sensor_parameters_t params)
{
	double aux[params.vertical_resolution];

	memcpy(aux, params.vertical_correction, params.vertical_resolution * sizeof(double));

	for (int i = 0; i < params.vertical_resolution; i++)
	{
		params.vertical_correction[i] = aux[params.ray_order[i]];
	}
}


static void
init_velodyne_points(spherical_point_cloud **velodyne_points_out, unsigned char ***intencity, carmen_pose_3D_t **robot_pose_out, carmen_vector_3D_t **robot_velocity_out, double **robot_timestamp_out, double **robot_phi_out)
{
	int i;

	carmen_pose_3D_t *robot_pose = (carmen_pose_3D_t *) calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_pose_3D_t));
	carmen_vector_3D_t *robot_velocity = (carmen_vector_3D_t *) calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_vector_3D_t));
	spherical_point_cloud *velodyne_points = (spherical_point_cloud *) malloc(NUM_VELODYNE_POINT_CLOUDS * sizeof(spherical_point_cloud));
	double *robot_timestamp = (double *) calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));
	*intencity = (unsigned char **) calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(unsigned char*));
	*robot_phi_out = (double *) calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));


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
get_alive_LIDARs_and_their_parameters(int argc, char **argv, int correction_type)
{
	int i;
	char sensor[128];
	char locc[128];
	char lfree[128];
	char l0[128];
	char unexpeted_delta_range_sigma[128];
	char range_max_factor[128];

	for (i = 10; i < number_of_sensors; i++)
	{
		sprintf(sensor, "lidar%d", i - 10);
		sprintf(locc, "lidar%d_locc", i - 10);
		sprintf(lfree, "lidar%d_lfree", i - 10);
		sprintf(l0, "lidar%d_l0", i - 10);
		sprintf(unexpeted_delta_range_sigma, "lidar%d_unexpeted_delta_range_sigma", i - 10);
		sprintf(range_max_factor, "lidar%d_range_max_factor", i - 10);

		// Inicio do carregamento dos parametros do sensor
		carmen_param_t param_list[] =
		{
			{(char *) "localize_ackerman", sensor, CARMEN_PARAM_ONOFF, &spherical_sensor_params[i].alive, 0, NULL},
			{(char *) "localize_ackerman", locc, CARMEN_PARAM_DOUBLE, &spherical_sensor_params[i].log_odds.log_odds_occ, 0, NULL},
			{(char *) "localize_ackerman", lfree, CARMEN_PARAM_DOUBLE, &spherical_sensor_params[i].log_odds.log_odds_free, 0, NULL},
			{(char *) "localize_ackerman", l0, CARMEN_PARAM_DOUBLE, &spherical_sensor_params[i].log_odds.log_odds_l0, 0, NULL},
			{(char *) "localize_ackerman", unexpeted_delta_range_sigma, CARMEN_PARAM_DOUBLE, &spherical_sensor_params[i].unexpeted_delta_range_sigma, 0, NULL},
			{(char *) "localize_ackerman", range_max_factor, CARMEN_PARAM_DOUBLE, &spherical_sensor_params[i].range_max_factor, 0, NULL},
			{(char *) "localize_ackerman", (char *) "unsafe_height_above_ground", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[i].unsafe_height_above_ground, 0, NULL},
		};
		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

		if (spherical_sensor_params[i].alive)
		{
			carmen_lidar_config *p;
			p = &lidar_config[i];
			load_lidar_config(argc, argv, i - 10, &p);

			spherical_sensor_params[i].name = lidar_config[i].model;
			spherical_sensor_params[i].pose = lidar_config[i].pose;
			spherical_sensor_params[i].sensor_support_pose = sensor_board_1_pose;
			spherical_sensor_params[i].support_to_car_matrix = create_rotation_matrix(spherical_sensor_params[i].sensor_support_pose.orientation);
			spherical_sensor_params[i].sensor_to_support_matrix = create_rotation_matrix(spherical_sensor_params[i].pose.orientation);
			spherical_sensor_params[i].sensor_robot_reference = carmen_change_sensor_reference(spherical_sensor_params[i].sensor_support_pose.position,
																spherical_sensor_params[i].pose.position, spherical_sensor_params[i].support_to_car_matrix);

			spherical_sensor_params[i].height = spherical_sensor_params[i].sensor_robot_reference.z + robot_wheel_radius;
			if (spherical_sensor_params[i].height > highest_sensor)
				highest_sensor = spherical_sensor_params[i].height;

			spherical_sensor_params[i].sensor_type = VELODYNE;
			spherical_sensor_params[i].vertical_resolution = lidar_config[i].shot_size;
			spherical_sensor_params[i].ray_order = lidar_config[i].ray_order;
			spherical_sensor_params[i].vertical_correction = lidar_config[i].vertical_angles;
			spherical_sensor_params[i].range_max = lidar_config[i].max_range;
			spherical_sensor_params[i].current_range_max = spherical_sensor_params[i].range_max;
			spherical_sensor_params[i].range_division_factor = lidar_config[i].range_division_factor;
			spherical_sensor_params[i].time_spent_by_each_scan = lidar_config[i].time_between_shots;

			int use_remission = (correction_type == 4) || (correction_type == 5) || (correction_type == 6) || (correction_type == 7); // See carmen_ford_escape.ini # TODO usar o correction type
			spherical_sensor_params[i].use_remission = use_remission;

			if (calibration_file)
				spherical_sensor_params[i].calibration_table = load_calibration_table(calibration_file);
			else
				spherical_sensor_params[i].calibration_table = load_calibration_table((char *) "calibration_table.txt");
			spherical_sensor_params[i].save_calibration_file = NULL;
			spherical_sensor_params[i].remission_calibration = NULL; //(double *) calloc(256 * sensors_params[0].vertical_resolution, sizeof(double));

			sort_ray_order_by_vertical_correction_angles(spherical_sensor_params[i]);
			sort_vertical_correction_angles(spherical_sensor_params[i]);
			// Fim do carregamento dos parametros do sensor

			// Inicio da alocacao de espaco para aos dados do sensor
			spherical_sensor_data[i].ray_position_in_the_floor = (carmen_vector_2D_t **)  calloc(number_of_threads, sizeof(carmen_vector_2D_t*));
			spherical_sensor_data[i].maxed = (int **) calloc(number_of_threads, sizeof(int*));
			spherical_sensor_data[i].obstacle_height = (double **) calloc(number_of_threads, sizeof(double*));
			spherical_sensor_data[i].occupancy_log_odds_of_each_ray_target = (double **) calloc(number_of_threads, sizeof(double*));
			spherical_sensor_data[i].point_cloud_index = -1;
			spherical_sensor_data[i].points = NULL;
			spherical_sensor_data[i].ray_origin_in_the_floor = (carmen_vector_2D_t **) calloc(number_of_threads, sizeof(carmen_vector_2D_t*));
			spherical_sensor_data[i].ray_size_in_the_floor = (double **) calloc(number_of_threads, sizeof(double*));
			spherical_sensor_data[i].processed_intensity = (double **) calloc(number_of_threads, sizeof(double*));
			spherical_sensor_data[i].ray_hit_the_robot = (int **) calloc(number_of_threads, sizeof(int*));
			spherical_sensor_data[i].ray_that_hit_the_nearest_target = (int *) calloc(number_of_threads, sizeof(int));

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

			init_velodyne_points(&spherical_sensor_data[i].points, &spherical_sensor_data[i].intensity, &spherical_sensor_data[i].robot_pose,
					&spherical_sensor_data[i].robot_velocity, &spherical_sensor_data[i].robot_timestamp, &spherical_sensor_data[i].robot_phi);

			spherical_sensor_data[i].point_cloud_index = -1;

			carmen_prob_models_alloc_sensor_data(&spherical_sensor_data[i], spherical_sensor_params[i].vertical_resolution, number_of_threads);
		}
	}
}


void
get_alive_sensors(int argc, char **argv)
{
	int i;

	spherical_sensor_params = (sensor_parameters_t *) calloc(number_of_sensors, sizeof(sensor_parameters_t));
	carmen_test_alloc(spherical_sensor_params);

	spherical_sensor_data = (sensor_data_t *) calloc(number_of_sensors, sizeof(sensor_data_t));
	carmen_test_alloc(spherical_sensor_data);

	carmen_param_t param_list[] =
	{
		{(char *) "localize_ackerman", (char *) "velodyne", CARMEN_PARAM_ONOFF, &spherical_sensor_params[0].alive, 0, NULL},
		{(char *) "localize_ackerman", (char *) "laser_ldmrs", CARMEN_PARAM_ONOFF, &spherical_sensor_params[1].alive, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne2", CARMEN_PARAM_ONOFF, &spherical_sensor_params[2].alive, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne3", CARMEN_PARAM_ONOFF, &spherical_sensor_params[3].alive, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne4", CARMEN_PARAM_ONOFF, &spherical_sensor_params[4].alive, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne5", CARMEN_PARAM_ONOFF, &spherical_sensor_params[5].alive, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne6", CARMEN_PARAM_ONOFF, &spherical_sensor_params[6].alive, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne7", CARMEN_PARAM_ONOFF, &spherical_sensor_params[7].alive, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne8", CARMEN_PARAM_ONOFF, &spherical_sensor_params[8].alive, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne9", CARMEN_PARAM_ONOFF, &spherical_sensor_params[9].alive, 0, NULL},

		{(char *) "localize_ackerman", (char *) "velodyne_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_occ, 0, NULL},
		{(char *) "localize_ackerman", (char *) "laser_ldmrs_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[1].log_odds.log_odds_occ, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne2_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].log_odds.log_odds_occ, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne3_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].log_odds.log_odds_occ, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne4_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].log_odds.log_odds_occ, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne5_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].log_odds.log_odds_occ, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne6_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].log_odds.log_odds_occ, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne7_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].log_odds.log_odds_occ, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne8_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].log_odds.log_odds_occ, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne9_locc", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].log_odds.log_odds_occ, 0, NULL},

		{(char *) "localize_ackerman", (char *) "velodyne_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_free, 0, NULL},
		{(char *) "localize_ackerman", (char *) "laser_ldmrs_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[1].log_odds.log_odds_free, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne2_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].log_odds.log_odds_free, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne3_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].log_odds.log_odds_free, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne4_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].log_odds.log_odds_free, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne5_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].log_odds.log_odds_free, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne6_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].log_odds.log_odds_free, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne7_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].log_odds.log_odds_free, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne8_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].log_odds.log_odds_free, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne9_lfree", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].log_odds.log_odds_free, 0, NULL},

		{(char *) "localize_ackerman", (char *) "velodyne_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].log_odds.log_odds_l0, 0, NULL},
		{(char *) "localize_ackerman", (char *) "laser_ldmrs_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[1].log_odds.log_odds_l0, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne2_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].log_odds.log_odds_l0, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne3_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].log_odds.log_odds_l0, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne4_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].log_odds.log_odds_l0, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne5_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].log_odds.log_odds_l0, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne6_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].log_odds.log_odds_l0, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne7_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].log_odds.log_odds_l0, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne8_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].log_odds.log_odds_l0, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne9_l0", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].log_odds.log_odds_l0, 0, NULL},

		{(char *) "localize_ackerman", (char *) "velodyne_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].unexpeted_delta_range_sigma, 0, NULL},
		{(char *) "localize_ackerman", (char *) "laser_ldmrs_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[1].unexpeted_delta_range_sigma, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne2_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[2].unexpeted_delta_range_sigma, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne3_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[3].unexpeted_delta_range_sigma, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne4_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[4].unexpeted_delta_range_sigma, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne5_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[5].unexpeted_delta_range_sigma, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne6_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[6].unexpeted_delta_range_sigma, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne7_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[7].unexpeted_delta_range_sigma, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne8_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[8].unexpeted_delta_range_sigma, 0, NULL},
		{(char *) "localize_ackerman", (char *) "stereo_velodyne9_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[9].unexpeted_delta_range_sigma, 0, NULL},

		{(char *) "localize_ackerman", (char *) "unsafe_height_above_ground", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].unsafe_height_above_ground, 0, NULL},

		{(char *) "localize_ackerman",  (char *) "velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[0].range_max_factor, 0, NULL}
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	// velodyne, ldmrs e stereo cameras
	for (i = 0; i < 10; i++)
	{
		if (i == STEREO_MAPPING_SENSOR_INDEX)
			continue;

		spherical_sensor_params[i].unsafe_height_above_ground = spherical_sensor_params[0].unsafe_height_above_ground;

		spherical_sensor_data[i].ray_position_in_the_floor = (carmen_vector_2D_t **) calloc(number_of_threads ,sizeof(carmen_vector_2D_t *));
		spherical_sensor_data[i].maxed = (int **) calloc(number_of_threads ,sizeof(int *));
		spherical_sensor_data[i].obstacle_height = (double **) calloc(number_of_threads ,sizeof(double *));
		spherical_sensor_data[i].occupancy_log_odds_of_each_ray_target = (double**)calloc(number_of_threads ,sizeof(double*));
		spherical_sensor_data[i].point_cloud_index = 0;
		spherical_sensor_data[i].points = NULL;
		spherical_sensor_data[i].ray_origin_in_the_floor = (carmen_vector_2D_t **)calloc(number_of_threads ,sizeof(carmen_vector_2D_t *));;
		spherical_sensor_data[i].ray_size_in_the_floor = (double **) calloc(number_of_threads ,sizeof(double *));
		spherical_sensor_data[i].processed_intensity = (double **) calloc(number_of_threads ,sizeof(double *));
		spherical_sensor_data[i].ray_hit_the_robot = (int **) calloc(number_of_threads ,sizeof(int *));
		spherical_sensor_data[i].ray_that_hit_the_nearest_target = (int *) calloc(number_of_threads ,sizeof(int));

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
			spherical_sensor_params[i].name = (char *) calloc(strlen(param_list[i].variable) + 1, sizeof(char));
			strcpy(spherical_sensor_params[i].name, param_list[i].variable);
		}
	}
}


static void
get_sensors_param(int argc, char **argv, int correction_type)
{
	int i;
	int flipped;
	int horizontal_resolution;

	int stereo_velodyne_vertical_roi_ini;
	int stereo_velodyne_vertical_roi_end;

	int stereo_velodyne_horizontal_roi_ini;
	int stereo_velodyne_horizontal_roi_end;

	int roi_ini, roi_end;

	int use_remission = (correction_type == 4) || (correction_type == 5) || (correction_type == 6) || (correction_type == 7); // See carmen_ford_escape.ini
	spherical_sensor_params[0].use_remission = use_remission;

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
		// spherical_sensor_params[0].delta_difference_mean = carmen_velodyne_get_delta_difference_mean();
		// spherical_sensor_params[0].delta_difference_stddev = carmen_velodyne_get_delta_difference_stddev();

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

//		spherical_sensor_params[0].remission_calibration = (double *) calloc(256 * spherical_sensor_params[0].vertical_resolution, sizeof(double));
//		FILE *f = fopen("../data/remission_calibration.txt", "r");
//		for (i = 0; i < 256 * spherical_sensor_params[0].vertical_resolution; i++)
//		{
//			fscanf(f, "%lf", &spherical_sensor_params[0].remission_calibration[i]);
//		}
//		fclose(f);

		spherical_sensor_params[0].current_range_max = spherical_sensor_params[0].range_max;
	}

	// Le parametros de stereo_velodyne. Lidars nao sao preenchidos aqui
	for (i = 1; i < 10; i++)
	{
		spherical_sensor_params[i].use_remission = use_remission;

		spherical_sensor_params[i].calibration_table = NULL;
		spherical_sensor_params[i].save_calibration_file = NULL;

		if (spherical_sensor_params[i].alive)
		{
			spherical_sensor_params[i].pose = get_stereo_velodyne_pose_3D(argc, argv, i);

			spherical_sensor_params[i].sensor_robot_reference = carmen_change_sensor_reference(sensor_board_1_pose.position, spherical_sensor_params[i].pose.position, sensor_board_1_to_car_matrix);
			spherical_sensor_params[i].height = spherical_sensor_params[i].sensor_robot_reference.z + robot_wheel_radius;

			if (spherical_sensor_params[i].height > highest_sensor)
				highest_sensor = spherical_sensor_params[i].height;

			carmen_param_t param_list[] =
			{
				{spherical_sensor_params[i].name, (char *) "vertical_resolution", CARMEN_PARAM_INT, &spherical_sensor_params[i].vertical_resolution, 0, NULL},
				{spherical_sensor_params[i].name, (char *) "horizontal_resolution", CARMEN_PARAM_INT, &horizontal_resolution, 0, NULL},
				{spherical_sensor_params[i].name, (char *) "flipped", CARMEN_PARAM_ONOFF, &flipped, 0, NULL},
				{spherical_sensor_params[i].name, (char *) "range_max", CARMEN_PARAM_DOUBLE, &spherical_sensor_params[i].range_max, 0, NULL},
				{spherical_sensor_params[i].name, (char *) "vertical_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_ini, 0, NULL },
				{spherical_sensor_params[i].name, (char *) "vertical_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_end, 0, NULL },
				{spherical_sensor_params[i].name, (char *) "horizontal_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_ini, 0, NULL },
				{spherical_sensor_params[i].name, (char *) "horizontal_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_end, 0, NULL }
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
				carmen_die("The stereo_velodyne_vertical_resolution is bigger than stereo point cloud height");

			spherical_sensor_params[i].current_range_max = spherical_sensor_params[i].range_max;

			spherical_sensor_params[i].range_max_factor = 1.0;
			spherical_sensor_params[i].ray_order = generates_ray_order(spherical_sensor_params[i].vertical_resolution);
			spherical_sensor_params[i].vertical_correction = get_stereo_velodyne_correction(flipped, i, spherical_sensor_params[i].vertical_resolution, roi_ini, roi_end, 0, 0);
			init_velodyne_points(&spherical_sensor_data[i].points, &spherical_sensor_data[i].intensity, &spherical_sensor_data[i].robot_pose,
					&spherical_sensor_data[i].robot_velocity, &spherical_sensor_data[i].robot_timestamp, &spherical_sensor_data[i].robot_phi);
			spherical_sensor_params[i].sensor_to_support_matrix = create_rotation_matrix(spherical_sensor_params[i].pose.orientation);
			spherical_sensor_data[i].point_cloud_index = 0;
			carmen_prob_models_alloc_sensor_data(&spherical_sensor_data[i], spherical_sensor_params[i].vertical_resolution, number_of_threads);

			//TODO : tem que fazer esta medida para as cameras igual foi feito para o velodyne
			// spherical_sensor_params[i].delta_difference_mean = (double *)calloc(50, sizeof(double));
			// spherical_sensor_params[i].delta_difference_stddev = (double *)calloc(50, sizeof(double));
			// for (j = 0; j < 50; j++)
			// 	spherical_sensor_params[i].delta_difference_stddev[j] = 1.0;
		}
	}
}


void
carmen_localize_ackerman_read_parameters(int argc, char **argv, carmen_localize_ackerman_param_p param,
		ProbabilisticMapParams *p_map_params)
{
	double integrate_angle_deg, map_width, map_height;

	integrate_angle_deg = 1.0;

	carmen_param_t param_list[] =
	{
		{(char *) "localize_ackerman", 	(char *) "velocity_noise_velocity", CARMEN_PARAM_DOUBLE, &param->velocity_noise_velocity, 0, NULL},
		{(char *) "localize_ackerman", 	(char *) "velocity_noise_phi", CARMEN_PARAM_DOUBLE, &param->velocity_noise_phi, 0, NULL},
		{(char *) "localize_ackerman", 	(char *) "phi_noise_phi", CARMEN_PARAM_DOUBLE, &param->phi_noise_phi, 0, NULL},
		{(char *) "localize_ackerman", 	(char *) "phi_noise_velocity", CARMEN_PARAM_DOUBLE, &param->phi_noise_velocity, 0, NULL},
		{(char *) "localize_ackerman", 	(char *) "prediction_type", CARMEN_PARAM_INT, &param->prediction_type, 0, NULL},

		{(char *) "robot", (char *) "frontlaser_offset", CARMEN_PARAM_DOUBLE, &param->front_laser_offset, 0, NULL},
		{(char *) "robot", (char *) "rearlaser_offset", CARMEN_PARAM_DOUBLE, &param->rear_laser_offset, 0, NULL},
		{(char *) "robot", (char *) "length", CARMEN_PARAM_DOUBLE, &car_config.length, 0, NULL},
		{(char *) "robot", (char *) "width", CARMEN_PARAM_DOUBLE, &car_config.width, 0, NULL},
		{(char *) "robot", (char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &car_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
		{(char *) "robot", (char *) "distance_between_front_and_rear_axles",		CARMEN_PARAM_DOUBLE, &car_config.distance_between_front_and_rear_axles, 1, NULL},
		{(char *) "robot", (char *) "publish_odometry", CARMEN_PARAM_DOUBLE, &robot_publish_odometry, 1, NULL},

		{(char *) "model", 		(char *) "predictive_planner_obstacles_safe_distance", 	CARMEN_PARAM_DOUBLE, &car_config.model_predictive_planner_obstacles_safe_distance, 1, NULL},
		{(char *) "obstacle", 	(char *) "avoider_obstacles_safe_distance", 			CARMEN_PARAM_DOUBLE, &car_config.obstacle_avoider_obstacles_safe_distance, 1, NULL},

		{(char *) "semi_trailer",	 (char *) "initial_type",						CARMEN_PARAM_INT, 	 &(semi_trailer_config.type), 					0, NULL},

		{(char *) "robot", (char *) "wheel_radius", CARMEN_PARAM_DOUBLE, &robot_wheel_radius, 0, NULL},

		{(char *) "localize", (char *) "use_rear_laser", CARMEN_PARAM_ONOFF, &param->use_rear_laser, 0, NULL},
		{(char *) "localize", (char *) "num_particles", CARMEN_PARAM_INT, &param->num_particles, 0, NULL},
		{(char *) "localize", (char *) "laser_max_range", CARMEN_PARAM_DOUBLE, &param->max_range, 1, NULL},
		{(char *) "localize", (char *) "min_wall_prob", CARMEN_PARAM_DOUBLE, &param->min_wall_prob, 0, NULL},
		{(char *) "localize", (char *) "outlier_fraction", CARMEN_PARAM_DOUBLE, &param->outlier_fraction, 0, NULL},
		{(char *) "localize", (char *) "update_distance", CARMEN_PARAM_DOUBLE, &param->update_distance, 0, NULL},
		{(char *) "localize", (char *) "integrate_angle_deg", CARMEN_PARAM_DOUBLE, &integrate_angle_deg, 0, NULL},
		{(char *) "localize", (char *) "do_scanmatching", CARMEN_PARAM_ONOFF, &param->do_scanmatching, 1, NULL},
		{(char *) "localize", (char *) "constrain_to_map", CARMEN_PARAM_ONOFF, &param->constrain_to_map, 1, NULL},
#ifdef OLD_MOTION_MODEL
		{(char *) "localize", (char *) "odom_a1", CARMEN_PARAM_DOUBLE, &param->odom_a1, 1, NULL},
		{(char *) "localize", (char *) "odom_a2", CARMEN_PARAM_DOUBLE, &param->odom_a2, 1, NULL},
		{(char *) "localize", (char *) "odom_a3", CARMEN_PARAM_DOUBLE, &param->odom_a3, 1, NULL},
		{(char *) "localize", (char *) "odom_a4", CARMEN_PARAM_DOUBLE, &param->odom_a4, 1, NULL},
#endif
		{(char *) "localize", (char *) "occupied_prob", CARMEN_PARAM_DOUBLE, &param->occupied_prob, 0, NULL},
		{(char *) "localize", (char *) "global_evidence_weight", CARMEN_PARAM_DOUBLE, &param->global_evidence_weight, 0, NULL},
		{(char *) "localize", (char *) "global_distance_threshold", CARMEN_PARAM_DOUBLE, &param->global_distance_threshold, 1, NULL},
		{(char *) "localize", (char *) "global_test_samples", CARMEN_PARAM_INT, &param->global_test_samples, 1, NULL},
		{(char *) "localize", (char *) "use_sensor", CARMEN_PARAM_ONOFF, &param->use_sensor, 0, NULL},
		{(char *) "localize", (char *) "phi_bias_std", CARMEN_PARAM_DOUBLE, &param->phi_bias_std, 0, NULL},
		{(char *) "localize", (char *) "lmap_std", CARMEN_PARAM_DOUBLE, &param->lmap_std, 0, NULL},
		{(char *) "localize", (char *) "global_lmap_std", CARMEN_PARAM_DOUBLE, &param->global_lmap_std, 0, NULL},
		{(char *) "localize", (char *) "use_log_odds", CARMEN_PARAM_ONOFF, &param->use_log_odds, 0, NULL},
		{(char *) "localize", (char *) "tracking_beam_minlikelihood", CARMEN_PARAM_DOUBLE, &param->tracking_beam_minlikelihood, 0, NULL},
		{(char *) "localize", (char *) "tracking_beam_maxlikelihood", CARMEN_PARAM_DOUBLE, &param->tracking_beam_maxlikelihood, 0, NULL},
		{(char *) "localize", (char *) "global_beam_minlikelihood", CARMEN_PARAM_DOUBLE, &param->global_beam_minlikelihood, 0, NULL},
		{(char *) "localize", (char *) "global_beam_maxlikelihood", CARMEN_PARAM_DOUBLE, &param->global_beam_maxlikelihood, 0, NULL},

		{(char *) "localize", (char *) "min_remission_variance", CARMEN_PARAM_DOUBLE, &param->min_remission_variance, 0, NULL},
		{(char *) "localize", (char *) "small_remission_likelihood", CARMEN_PARAM_DOUBLE, &param->small_remission_likelihood, 0, NULL},

		{(char *) "localize", (char *) "particles_normalize_factor", CARMEN_PARAM_DOUBLE, &param->particles_normalize_factor, 0, NULL},

		{(char *) "localize", (char *) "yaw_uncertainty_due_to_grid_resolution", CARMEN_PARAM_DOUBLE, &param->yaw_uncertainty_due_to_grid_resolution, 0, NULL},
		{(char *) "localize", (char *) "xy_uncertainty_due_to_grid_resolution", CARMEN_PARAM_DOUBLE, &param->xy_uncertainty_due_to_grid_resolution, 0, NULL},

		{(char *) "localize", (char *) "v_uncertainty_at_zero_v", CARMEN_PARAM_DOUBLE, &param->v_uncertainty_at_zero_v, 0, NULL},

		{(char *) "localize", (char *) "remission_variance_multiplier", CARMEN_PARAM_DOUBLE, &param->remission_variance_multiplier, 0, NULL},

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

		{(char *) "localize_ackerman", (char *) "number_of_sensors", CARMEN_PARAM_INT, &number_of_sensors, 0, NULL},
		{(char *) "localize_ackerman", (char *) "safe_range_above_sensors", CARMEN_PARAM_DOUBLE, &safe_range_above_sensors, 0, NULL},
		{(char *) "localize_ackerman", (char *) "correction_type", CARMEN_PARAM_INT, &param->correction_type, 0, NULL},

		{(char *) "localize_ackerman", (char *) "swarm_max_particle_velocity", CARMEN_PARAM_DOUBLE, &param->swarm_max_particle_velocity, 0, NULL},
		{(char *) "localize_ackerman", (char *) "swarm_max_particle_angular_velocity", CARMEN_PARAM_DOUBLE, &param->swarm_max_particle_angular_velocity, 0, NULL},
		{(char *) "localize_ackerman", (char *) "max_particle_displacement", CARMEN_PARAM_DOUBLE, &param->max_particle_displacement, 0, NULL},
		{(char *) "localize_ackerman", (char *) "max_particle_angular_displacement", CARMEN_PARAM_DOUBLE, &param->max_particle_angular_displacement, 0, NULL},
		{(char *) "localize_ackerman", (char *) "de_crossover_rate", CARMEN_PARAM_DOUBLE, &param->de_crossover_rate, 0, NULL},
		{(char *) "localize_ackerman", (char *) "de_mutation_rate", CARMEN_PARAM_DOUBLE, &param->de_mutation_rate, 0, NULL},
		{(char *) "localize_ackerman", (char *) "de_num_iteration", CARMEN_PARAM_INT, &param->de_num_iteration, 0, NULL},
		{(char *) "localize_ackerman", (char *) "swarm_num_iteration", CARMEN_PARAM_INT, &param->swarm_num_iteration, 0, NULL},
		{(char *) "localize_ackerman", (char *) "jump_size", CARMEN_PARAM_INT, &param->jump_size, 0, NULL},

		{(char *) "mapper", (char *) "map_log_odds_max", CARMEN_PARAM_INT, &p_map_params->log_odds_max, 0, NULL},
		{(char *) "mapper", (char *) "map_log_odds_min", CARMEN_PARAM_INT, &p_map_params->log_odds_min, 0, NULL},
		{(char *) "mapper", (char *) "map_log_odds_bias", CARMEN_PARAM_INT, &p_map_params->log_odds_bias, 0, NULL},
		{(char *) "mapper", (char *) "map_grid_res", CARMEN_PARAM_DOUBLE, &p_map_params->grid_res, 0, NULL},
		{(char *) "mapper", (char *) "map_range_factor", CARMEN_PARAM_DOUBLE, &p_map_params->range_factor, 0, NULL},
		{(char *) "map_server", (char*) "map_width", CARMEN_PARAM_DOUBLE, &map_width, 0, NULL},
		{(char *) "map_server", (char*) "map_height", CARMEN_PARAM_DOUBLE, &map_height, 0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	if (semi_trailer_config.type > 0)
		carmen_task_manager_read_semi_trailer_parameters(&semi_trailer_config, argc, argv, semi_trailer_config.type);

	carmen_param_allow_unfound_variables(1);

	carmen_param_t param_optional_list[] =
	{
		{(char *) "localize_ackerman", (char *) "use_raw_laser", CARMEN_PARAM_ONOFF, &use_raw_laser, 0, NULL},
		{(char *) "commandline", (char *) "mapping_mode", CARMEN_PARAM_ONOFF, &mapping_mode, 0, NULL},
		{(char *) "commandline", (char *) "velodyne_viewer", CARMEN_PARAM_ONOFF, &velodyne_viewer, 0, NULL},
		{(char *) "commandline", (char *) "calibration_file", CARMEN_PARAM_STRING, &calibration_file, 0, NULL},
		{(char *) "commandline", (char *) "save_globalpos_file", CARMEN_PARAM_STRING, &save_globalpos_file, 0, NULL},
		{(char *) "commandline", (char *) "save_globalpos_timestamp", CARMEN_PARAM_DOUBLE, &save_globalpos_timestamp, 0, NULL}
	};

	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));

	param->integrate_angle = carmen_degrees_to_radians(integrate_angle_deg);

	sensor_board_1_to_car_matrix = create_rotation_matrix(sensor_board_1_pose.orientation);

	get_alive_sensors(argc, argv);
	get_sensors_param(argc, argv, param->correction_type);
	get_alive_LIDARs_and_their_parameters(argc, argv, param->correction_type);

	p_map_params->width = map_width;
	p_map_params->height = map_height;
	p_map_params->grid_sx = p_map_params->width /  p_map_params->grid_res;
	p_map_params->grid_sy = p_map_params->height /  p_map_params->grid_res;
	p_map_params->grid_size = p_map_params->grid_sx * p_map_params->grid_sy;

//	carmen_map_config_t main_map_config;
//
//	main_map_config.resolution = p_map_params->grid_res;
//	main_map_config.x_size = map_width / p_map_params->grid_res;
//	main_map_config.y_size = map_height / p_map_params->grid_res;
//
//	localize_using_map_initialize(&main_map_config);

	localize_ackerman_velodyne_laser_read_parameters(argc, argv);

	param->yaw_uncertainty_due_to_grid_resolution = carmen_degrees_to_radians(param->yaw_uncertainty_due_to_grid_resolution);
}

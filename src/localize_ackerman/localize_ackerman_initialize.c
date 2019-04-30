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
#include "localize_ackerman_interface.h"
#include "localize_ackerman_core.h"
#include "localize_ackerman_motion.h"
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <omp.h>

/* priority queue for sorting global localization hypotheses */

typedef struct queue_node {
	carmen_point_t point;
	float prob;
	struct queue_node *next, *prev;
} queue_node_t, *queue_node_p;

typedef struct {
	int num_elements, max_elements;
	queue_node_p first, last;
} priority_queue_t, *priority_queue_p;

/* initialize a new priority queue */


static priority_queue_p
priority_queue_init(int max_elements)
{
	priority_queue_p result;

	result = (priority_queue_p) calloc(1, sizeof(priority_queue_t));
	carmen_test_alloc(result);
	result->num_elements = 0;
	result->max_elements = max_elements;
	result->first = NULL;
	result->last = NULL;
	return result;
}

/* add a point to the priority queue */

static void
priority_queue_add(priority_queue_p queue, carmen_point_t point, float prob)
{
	queue_node_p mark, temp;

	if(queue->num_elements == 0) {
		temp = (queue_node_p)calloc(1, sizeof(queue_node_t));
		carmen_test_alloc(temp);
		temp->point = point;
		temp->prob = prob;
		temp->prev = NULL;
		temp->next = NULL;
		queue->first = temp;
		queue->last = temp;
		queue->num_elements++;
	}
	else if(prob > queue->last->prob ||
			queue->num_elements < queue->max_elements) {
		mark = queue->last;
		while(mark != NULL && prob > mark->prob)
			mark = mark->prev;
		if(mark == NULL) {
			temp = (queue_node_p)calloc(1, sizeof(queue_node_t));
			carmen_test_alloc(temp);
			temp->point = point;
			temp->prob = prob;
			temp->prev = NULL;
			temp->next = queue->first;
			queue->first->prev = temp;
			queue->first = temp;
			queue->num_elements++;
		}
		else {
			temp = (queue_node_p)calloc(1, sizeof(queue_node_t));
			carmen_test_alloc(temp);
			temp->point = point;
			temp->prob = prob;
			temp->prev = mark;
			temp->next = mark->next;
			if(mark->next != NULL)
				mark->next->prev = temp;
			else
				queue->last = temp;
			mark->next = temp;
			queue->num_elements++;
		}
		if(queue->num_elements > queue->max_elements) {
			queue->last = queue->last->prev;
			free(queue->last->next);
			queue->last->next = NULL;
			queue->num_elements--;
		}
	}
}

/* free the priority queue */

static void
priority_queue_free(priority_queue_p queue)
{
	queue_node_p mark;

	while(queue->first != NULL) {
		mark = queue->first;
		queue->first = queue->first->next;
		free(mark);
	}
	free(queue);
}


static void
initialize_temp_weights(carmen_localize_ackerman_particle_filter_p filter)
{
	int i;

	filter->temp_weights = (double **) calloc(filter->param->num_particles, sizeof(double *));
	carmen_test_alloc(filter->temp_weights);
	for (i = 0; i < filter->param->num_particles; i++)
	{
		filter->temp_weights[i] = (double *) calloc(MAX_BEAMS_PER_SCAN, sizeof(double));
		carmen_test_alloc(filter->temp_weights[i]);
	}
}


static void
realloc_temp_weights(carmen_localize_ackerman_particle_filter_p filter, int num_particles)
{
	int i;

	for (i = 0; i < filter->param->num_particles; i++)
		free(filter->temp_weights[i]);

	filter->temp_weights = (double **) realloc(filter->temp_weights, num_particles * sizeof(double *));
	carmen_test_alloc(filter->temp_weights);
	for (i = 0; i < num_particles; i++)
	{
		filter->temp_weights[i] = (double *)calloc(MAX_BEAMS_PER_SCAN, sizeof(double));
		carmen_test_alloc(filter->temp_weights[i]);
	}
}


carmen_localize_ackerman_particle_filter_p
carmen_localize_ackerman_particle_filter_initialize(carmen_localize_ackerman_param_p param)
{
	carmen_localize_ackerman_particle_filter_p filter;

	carmen_set_random_seed(time(NULL));

	filter = (carmen_localize_ackerman_particle_filter_p) calloc(1, sizeof(carmen_localize_ackerman_particle_filter_t));
	carmen_test_alloc(filter);

	filter->param = param;

	filter->particles = (carmen_localize_ackerman_particle_ipc_p) calloc(filter->param->num_particles, sizeof(carmen_localize_ackerman_particle_ipc_t));
	carmen_test_alloc(filter->particles);

	filter->swarm_pbest = (carmen_localize_ackerman_particle_ipc_p) calloc(filter->param->num_particles, sizeof(carmen_localize_ackerman_particle_ipc_t));
	carmen_test_alloc(filter->swarm_pbest);

	filter->swarm_velocity = (carmen_localize_ackerman_particle_ipc_p) calloc(filter->param->num_particles, sizeof(carmen_localize_ackerman_particle_ipc_t));
	carmen_test_alloc(filter->swarm_velocity);

	initialize_temp_weights(filter);

	filter->initialized = 0;
	filter->first_odometry = 1;
	filter->converged = 0;
	filter->distance_travelled = 0;

	filter->param->laser_skip = 0; /* will be automatically initialized later on */

	return filter;
}


void
carmen_localize_ackerman_initialize_particles_uniform(carmen_localize_ackerman_particle_filter_p filter,
		carmen_robot_ackerman_laser_message *laser,
		carmen_localize_ackerman_map_p map)
{
	priority_queue_p queue = priority_queue_init(filter->param->num_particles);
	double *laser_x, *laser_y;
	int i, j, x_l, y_l;
	float angle, prob, ctheta, stheta;
	carmen_point_t point;
	queue_node_p mark;
	int *beam_valid;


	/* compute the correct laser_skip */
	if (filter->param->laser_skip <= 0)
		filter->param->laser_skip = floor(filter->param->integrate_angle / laser->config.angular_resolution);

	fprintf(stderr, "\rDoing global localization... (%.1f%% complete)", 0.0);
	filter->initialized = 0;
	/* copy laser scan into temporary memory */
	laser_x = (double *)calloc(laser->num_readings, sizeof(double));
	carmen_test_alloc(laser_x);
	laser_y = (double *)calloc(laser->num_readings, sizeof(double));
	carmen_test_alloc(laser_y);
	beam_valid = (int *)calloc(laser->num_readings, sizeof(int));
	carmen_test_alloc(beam_valid);

	for(i = 0; i < laser->num_readings; i++) {
		if (laser->range[i] < laser->config.maximum_range &&
				laser->range[i] < filter->param->max_range)
			beam_valid[i] = 1;
		else
			beam_valid[i] = 0;
	}

	/* do all calculations in map coordinates */
	for(i = 0; i < laser->num_readings; i++) {
		angle = laser->config.start_angle +
				i * laser->config.angular_resolution;

		laser_x[i] = (filter->param->front_laser_offset +
				laser->range[i] * cos(angle)) / map->config.resolution;
		laser_y[i] = (laser->range[i] * sin(angle)) / map->config.resolution;
	}

	for(i = 0; i < filter->param->global_test_samples; i++) {
		if(i % 10000 == 0)
		{
			fprintf(stderr, "\rDoing global localization... (%.1f%% complete)",
					i / (double)filter->param->global_test_samples * 100.0);
			carmen_ipc_sleep(0.001);
		}
		do {
			point.x = carmen_uniform_random(0, map->config.x_size - 1);
			point.y = carmen_uniform_random(0, map->config.y_size - 1);
		} while(map->carmen_map.map[(int)point.x][(int)point.y] >
		filter->param->occupied_prob ||
		map->carmen_map.map[(int)point.x][(int)point.y] == -1);
		point.theta = carmen_uniform_random(-M_PI, M_PI);

		prob = 0.0;
		ctheta = cos(point.theta);
		stheta = sin(point.theta);
		for(j = 0; j < laser->num_readings &&
		(queue->last == NULL || prob > queue->last->prob);
		j += filter->param->laser_skip)
		{

			if (beam_valid[j]) {
				x_l = point.x + laser_x[j] * ctheta - laser_y[j] * stheta;
				y_l = point.y + laser_x[j] * stheta + laser_y[j] * ctheta;

				if(x_l >= 0 && y_l >= 0 && x_l < map->config.x_size &&
						y_l < map->config.y_size)
					prob += map->gprob[x_l][y_l];
				else
					prob -= 100;
			}
		}
		priority_queue_add(queue, point, prob);
	}

	/* transfer samples from priority queue back into particles */
	mark = queue->first;
	for(i = 0; i < queue->num_elements; i++) {
		filter->particles[i].x = (mark->point.x * map->config.resolution) + map->config.x_origin;
		filter->particles[i].y = (mark->point.y * map->config.resolution) + map->config.y_origin;
		filter->particles[i].theta = mark->point.theta;
		mark = mark->next;
	}
	priority_queue_free(queue);
	free(laser_x);
	free(laser_y);
	free(beam_valid);


	if(filter->param->do_scanmatching) {
		for(i = 0; i < filter->param->num_particles; i++) {
			point.x = filter->particles[i].x;
			point.y = filter->particles[i].y;
			point.theta = filter->particles[i].theta;
			carmen_localize_ackerman_laser_scan_gd(laser->num_readings, laser->range,
					laser->config.angular_resolution,
					laser->config.start_angle,
					&point,
					filter->param->front_laser_offset,
					map,
					filter->param->laser_skip);
			filter->particles[i].x = point.x;
			filter->particles[i].y = point.y;
			filter->particles[i].theta = point.theta;
			filter->particles[i].weight = 0.0;
		}
	}
	filter->initialized = 1;
	filter->first_odometry = 1;
	filter->converged = 1;
	filter->distance_travelled = 0;
	fprintf(stderr, "\rDoing global localization... (%.1f%% complete)\n\n",
			100.0);
}


void
carmen_localize_ackerman_initialize_particles_gaussians(carmen_localize_ackerman_particle_filter_p filter,
		int num_modes, carmen_point_t *mean, carmen_point_t *std)
{
	int i, j, n_particles_per_mode, start, end;
	double x, y, theta, phi_bias;

	// Currently, num_modes is always equals to 1. See localize_ackerman_interface.cpp, function 'carmen_localize_ackerman_initialize_gaussian_time_command'.
	// This initialization function is called in xsens_xyz_handler.cpp (fused_odometry module), function 'globalpos_ackerman_initialize_from_xsens'.
	n_particles_per_mode = (int) floor(filter->param->num_particles / (double) num_modes);

	for (i = 0; i < num_modes; i++)
	{
		start = i * n_particles_per_mode;

		// num particles is not necessarily multiple of num modes.
		if(i == num_modes - 1)
			end = filter->param->num_particles;
		else
			end = (i + 1) * n_particles_per_mode;

		for (j = start; j < end; j++)
		{
			x = carmen_gaussian_random(mean[i].x, std[i].x);
			y = carmen_gaussian_random(mean[i].y, std[i].y);
			theta = carmen_normalize_theta(carmen_gaussian_random(mean[i].theta, std[i].theta));
			phi_bias = carmen_gaussian_random(0.0, filter->param->phi_bias_std);

			filter->particles[j].x = x;
			filter->particles[j].y = y;
			filter->particles[j].theta = theta;
			filter->particles[j].phi_bias = phi_bias;
			filter->particles[j].weight = 0.5;
		}
	}

	// Add mean of each mode to the pool
	for (i = 0; i < num_modes; i++)
	{
		x = mean[i].x;
		y = mean[i].y;
		theta = carmen_normalize_theta(mean[i].theta);

		filter->particles[i].x = x;
		filter->particles[i].y = y;
		filter->particles[i].theta = theta;
		filter->particles[i].weight = 0.5;
		filter->particles[i].phi_bias = 0.0;
	}

	filter->initialized = 1;
	filter->first_odometry = 1;

	//	if (num_modes < 2)
	if (0)
		filter->converged = 0;
	else
		filter->converged = 1;

	filter->distance_travelled = 0;
}


int
carmen_localize_ackerman_initialize_particles_placename(carmen_localize_ackerman_particle_filter_p filter,
		carmen_map_placelist_p placelist,
		char *placename)
{
	carmen_point_t mean, std;

	int i;
	for(i = 0; i < placelist->num_places; i++)
		if(strcmp(placename, placelist->places[i].name) == 0)
			break;
	/*   if(i == placelist->num_places ||  */
	/*      placelist->places[i].type != CARMEN_LCALIZATION_INIT_TYPE) */
	/*     return -1; */
	if(i == placelist->num_places/*  ||  */
	/*      placelist->places[i].type != CARMEN_LCALIZATION_INIT_TYPE */)
		return -1;
	mean.x = placelist->places[i].x;
	mean.y = placelist->places[i].y;
	mean.theta = placelist->places[i].theta;
	std.x = placelist->places[i].x_std;
	std.y = placelist->places[i].y_std;
	std.theta = placelist->places[i].theta_std;
	carmen_localize_ackerman_initialize_particles_gaussian(filter, mean, std);
	return 0;
}

/* initialize particles from a gaussian distribution */
void
carmen_localize_ackerman_initialize_particles_gaussian(carmen_localize_ackerman_particle_filter_p filter,
		carmen_point_t mean,
		carmen_point_t std)
{
	carmen_localize_ackerman_initialize_particles_gaussians(filter, 1, &mean, &std);
}

/* initialize particle positions and weights from parameters */
void
carmen_localize_ackerman_initialize_particles_manual(carmen_localize_ackerman_particle_filter_p filter,
		double *x, double *y, double *theta,
		double *weight, int num_particles)
{
	int i;

	if(num_particles != filter->param->num_particles) {
		filter->particles =
				(carmen_localize_ackerman_particle_ipc_p)realloc(filter->particles, num_particles *
						sizeof(carmen_localize_ackerman_particle_ipc_t));
		carmen_test_alloc(filter->particles);
		realloc_temp_weights(filter, num_particles);
		filter->param->num_particles = num_particles;
	}
	for(i = 0; i < filter->param->num_particles; i++) {
		filter->particles[i].x = x[i];
		filter->particles[i].y = y[i];
		filter->particles[i].theta = theta[i];
		filter->particles[i].weight = weight[i];
	}
	filter->initialized = 1;
	filter->first_odometry = 1;
	filter->converged = 0;
	filter->distance_travelled = 0;
}


#include "prob_monte_carlo.h"
#include "prob_transforms.h"
#include "prob_motion_model.h"


/**
 *  initialize particles from a gaussian distribution
 */
void 
init_particles_from_gaussians(carmen_point_t *particles, carmen_point_t *_particles, double *weight,
		carmen_point_t *mean,
		carmen_point_t *std,
		int number_of_distribution_modes,
		int number_of_particles)
{
	int i, j, each, start, end;
	double x, y, theta;

	each = (int) floor(number_of_particles / (float) number_of_distribution_modes);
	for (i = 0; i < number_of_distribution_modes; i++)
	{
		start = i * each;
		if (i == number_of_distribution_modes - 1)
			end = number_of_particles;
		else
			end = (i + 1) * each;

		for (j = start; j < end; j++)
		{
			x = carmen_gaussian_random(mean[i].x, std[i].x);
			y = carmen_gaussian_random(mean[i].y, std[i].y);
			theta = carmen_normalize_theta(carmen_gaussian_random(mean[i].theta, std[i].theta));

			particles[j].x = x;
			particles[j].y = y;
			particles[j].theta = theta;

			_particles[j].x = x;
			_particles[j].y = y;
			_particles[j].theta = theta;
			
			weight[j] = 1.0 / number_of_particles;
		}
	}
}


void
build_particles_ackerman_message(carmen_localize_ackerman_particle_message *particles_ackerman_message,
		const carmen_localize_ackerman_globalpos_message *globalpos_ackerman_message,
		const int num_particles, const carmen_point_t *particles, const double *weights, double timestamp)
{
	particles_ackerman_message->globalpos = globalpos_ackerman_message->globalpos;
	particles_ackerman_message->globalpos_std = globalpos_ackerman_message->globalpos_std;
	for (int i=0; i < num_particles; i++)
	{
		particles_ackerman_message->particles[i].x = particles[i].x;
		particles_ackerman_message->particles[i].y = particles[i].y;
		particles_ackerman_message->particles[i].theta = particles[i].theta;
		particles_ackerman_message->particles[i].weight = weights[i];
	}
	
	particles_ackerman_message->timestamp = timestamp;
}


void
build_globalpos_ackerman_message(carmen_localize_ackerman_globalpos_message *globalpos_ackerman_message,
		const carmen_point_t *particles, const double *weights, const int num_particles,
		carmen_point_t odometry_pose, double v, double phi, int converged, double timestamp)
{
	double mean_x, mean_y, mean_theta_x, mean_theta_y;
	double diff_x, diff_y, diff_theta, std_x, std_y, std_theta, xy_cov;
	double *weights_normalized, max_weight = weights[0];
	double total_weight = 0;
	int i;

	globalpos_ackerman_message->odometrypos = odometry_pose;
	globalpos_ackerman_message->v = v;
	globalpos_ackerman_message->phi = phi;
	
	weights_normalized = (double *)calloc(num_particles, sizeof(double));
	carmen_test_alloc(weights_normalized);

	for (i = 0; i < num_particles; i++)
		if (weights[i] > max_weight)
			max_weight = weights[i];

	for (i = 0; i < num_particles; i++)
	{
		weights_normalized[i] = exp(weights[i] - max_weight);
		total_weight += weights_normalized[i];
	}

	/* compute mean particle pose */
	mean_x = 0;
	mean_y = 0;
	mean_theta_x = 0;
	mean_theta_y = 0;

	for (i = 0; i < num_particles; i++)
	{
		mean_x += particles[i].x * weights_normalized[i];
		mean_y += particles[i].y * weights_normalized[i];
		mean_theta_x += cos(particles[i].theta) * weights_normalized[i];
		mean_theta_y += sin(particles[i].theta) * weights_normalized[i];
	}
	globalpos_ackerman_message->globalpos.x = mean_x / total_weight;
	globalpos_ackerman_message->globalpos.y = mean_y / total_weight;

	if (mean_theta_x == 0)
		globalpos_ackerman_message->globalpos.theta = 0;
	else
		globalpos_ackerman_message->globalpos.theta = atan2(mean_theta_y, mean_theta_x);

	/* compute std particle pose */
	std_x = 0;
	std_y = 0;
	std_theta = 0;
	xy_cov = 0;
	for (i = 0; i < num_particles; i++)
	{
		diff_x = (particles[i].x - globalpos_ackerman_message->globalpos.x);
		diff_y = (particles[i].y - globalpos_ackerman_message->globalpos.y);
		diff_theta = carmen_normalize_theta(particles[i].theta - globalpos_ackerman_message->globalpos.theta);
		std_x += carmen_square(diff_x);
		std_y += carmen_square(diff_y);
		std_theta += carmen_square(diff_theta);
		xy_cov += diff_x * diff_y;
	}
	globalpos_ackerman_message->globalpos_std.x = sqrt(std_x / num_particles);
	globalpos_ackerman_message->globalpos_std.y = sqrt(std_y / num_particles);
	globalpos_ackerman_message->globalpos_std.theta = sqrt(std_theta / num_particles);
	globalpos_ackerman_message->globalpos_xy_cov = sqrt(xy_cov / num_particles);
	globalpos_ackerman_message->converged = converged;

	globalpos_ackerman_message->timestamp = timestamp;
	
	free(weights_normalized);
}


void
build_sensor_ackerman_message2(carmen_localize_ackerman_sensor_message *sensor_ackerman_message,const carmen_point_t *robot_pose,
		float *range, int num_readings, carmen_laser_laser_config_t laser_config, double timestamp)
{
	int k;

	sensor_ackerman_message->config = laser_config;

	for (k = 0; k < num_readings; k++)
	{
		sensor_ackerman_message->range[k] = range[k];
		sensor_ackerman_message->mask[k] = (range[k] >= laser_config.maximum_range)? 0: 1;
	}
	transform_robot_pose_to_laser_pose(&(sensor_ackerman_message->pose), robot_pose);

	sensor_ackerman_message->timestamp = timestamp;
}


void
build_sensor_ackerman_message(carmen_localize_ackerman_sensor_message *sensor_ackerman_message, const carmen_point_t *robot_pose, const carmen_robot_ackerman_laser_message *laser, double timestamp)
{
	int k;

	sensor_ackerman_message->config = laser->config;

	for (k = 0; k < laser->num_readings; k++)
	{
		sensor_ackerman_message->range[k] = laser->range[k];
		sensor_ackerman_message->mask[k] = (laser->range[k] >= laser->config.maximum_range)? 0: 1;
	}
	transform_robot_pose_to_laser_pose(&(sensor_ackerman_message->pose), robot_pose);

	sensor_ackerman_message->timestamp = timestamp;
}


carmen_point_t
improved_proposal_distribution_ackerman(const AckermanMotionCommand *ut, carmen_point_t xt_1,
		carmen_map_t* map, double *zt,
		int number_of_particles)
{
	int m;
	carmen_point_t _xt;
	double wt;
	double max_wt = -1.0;
	carmen_point_t max_xt;

	max_xt = xt_1;
	for (m = 0; m < number_of_particles; m++)
	{
		_xt = sample_motion_model_ackerman(ut, xt_1, 1);
		wt = carmen_beam_range_finder_measurement_model(zt, &_xt, map);
		//wt = 1.0; // uncomment this to disable correction

		if (wt > max_wt)
		{
			max_wt = wt;
			max_xt = _xt;
		}
	}

	return (max_xt);
}

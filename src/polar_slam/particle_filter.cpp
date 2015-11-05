/*
 * particle_filter.cpp
 *
 *  Created on: Dec 18, 2012
 *      Author: filipe mutz
 */
#include "particle_filter.h"
#include <prob_motion_model.h>
#include <prob_measurement_model.h>


Particle::Particle(int num_spheres, int num_angular_sections, int num_points_to_store)
{
	map = PolarMap(num_spheres, num_angular_sections, num_points_to_store);

	memset(&pose, 0, sizeof(pose));
	memset(&last_pose, 0, sizeof(last_pose));

	weight = 0;
}


Particle::Particle(const Particle &p)
{
	map = p.map;
	weight = p.weight;

	pose = p.pose;
	last_pose = p.last_pose;
}


Particle::~Particle()
{
}


ParticleFilter::ParticleFilter()
{
}


ParticleFilter::ParticleFilter(int num_particles, int num_spheres, int num_angular_sections, int num_points_to_store)
{
	first_map_has_been_received = 0;
	_num_particles = num_particles;

	for(int i = 0; i < num_particles; i++)
		particles.push_back(Particle(num_spheres, num_angular_sections, num_points_to_store));
}


ParticleFilter::~ParticleFilter()
{
}


carmen_pose_3D_t
ParticleFilter::generate_random_pose(carmen_pose_3D_t mean, carmen_pose_3D_t std)
{
	carmen_pose_3D_t pose;

	pose.position.x = carmen_gaussian_random(mean.position.x, std.position.x);
	pose.position.y = carmen_gaussian_random(mean.position.y, std.position.y);
	pose.position.z = carmen_gaussian_random(mean.position.z, std.position.z);

	pose.orientation.roll = carmen_normalize_theta(carmen_gaussian_random(mean.orientation.roll, std.orientation.roll));
	pose.orientation.pitch = carmen_normalize_theta(carmen_gaussian_random(mean.orientation.pitch, std.orientation.pitch));
	pose.orientation.yaw = carmen_normalize_theta(carmen_gaussian_random(mean.orientation.yaw, std.orientation.yaw));

	return pose;
}


void
ParticleFilter::generate_particles(carmen_pose_3D_t mean, carmen_pose_3D_t std)
{
	for(unsigned int i = 0; i < particles.size(); i++)
	{
		particles[i].pose = generate_random_pose(mean, std);
		particles[i].last_pose = particles[i].pose;
	}
}


void
ParticleFilter::predict_pose_using_motion_model(OdometryMotionCommand *ut)
{
	carmen_point_t last_particle_pose, predicted_particle_pose;

	for(unsigned int i = 0; i < particles.size(); i++)
	{
		last_particle_pose.x = particles[i].pose.position.x;
		last_particle_pose.y = particles[i].pose.position.y;
		last_particle_pose.theta = particles[i].pose.orientation.yaw;

		predicted_particle_pose = sample_motion_model_odometry(ut, last_particle_pose);

		particles[i].pose.position.x = predicted_particle_pose.x;
		particles[i].pose.position.y = predicted_particle_pose.y;
		particles[i].pose.orientation.yaw = predicted_particle_pose.theta;
	}
}


double
ParticleFilter::calculate_particle_probability(Particle &particle, carmen_laser_laser_message *laser_message)
{
	double angle, sensor_real_measurement, sensor_measurement_estimated_from_map, particle_weight, ray_weight;

	double starting_angle = laser_message->config.start_angle;
	double angle_variation = laser_message->config.angular_resolution;

	particle_weight = 1.0;

	for(int i = 0; i < laser_message->num_readings; i++)
	{
		angle = carmen_normalize_theta(starting_angle + i * angle_variation);

		sensor_real_measurement = laser_message->range[i];
		sensor_measurement_estimated_from_map = particle.map.ray_cast(angle);

		if (sensor_measurement_estimated_from_map < 0)
			sensor_measurement_estimated_from_map = laser_message->config.maximum_range;

		ray_weight = beam_range_finder_model_probability(sensor_measurement_estimated_from_map, sensor_real_measurement);

		//printf("Ops! Ray weight = %lf in the beam %d particle weight: %.15lf\n", ray_weight, i, particle_weight);

		particle_weight = particle_weight * (200 * ray_weight);
	}

	return particle_weight;
}

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// esse metodo esta com problema! ele gera um bad_alloc.
// pelo que li na internet isso parece um stack overflow
// causado pela copia de granes quantidades de dados
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void
ParticleFilter::resample_particles()
{
	//
	// this method implements the low_variance_sampler presented by Thrun (see Probabilistic Robotics, p. 110)
	//
	vector<Particle> resampled_particles;

	srand(time(NULL));

	unsigned int i = 0;
	unsigned int num_particles = particles.size();

	double M_inv = 1.0 / ((double) num_particles);
	double r = M_inv * ((double) rand() / (double) RAND_MAX);
	double c = particles[0].weight;

	for(unsigned int m = 0; m < num_particles; m++)
	{
		double U = r + m * M_inv;

		while (U > c)
		{
			c += particles[i].weight;
			i++;
		}

		if (i > particles.size())
		{
			i = rand() % particles.size();
			printf("Choosen particle index is bigger than particle list length in the resample\n");
		}

		resampled_particles.push_back(Particle(particles[i]));
	}

	particles = resampled_particles;

	for(i = 0; i < particles.size(); i++)
		particles[i].weight = M_inv;
}


void
ParticleFilter::normalize_particle_weights()
{
	double particle_summed_weight = 0;

	for(unsigned int i = 0; i < particles.size(); i++)
		particle_summed_weight += particles[i].weight;

	for(unsigned int i = 0; i < particles.size(); i++)
		particles[i].weight /= particle_summed_weight;
}


void
ParticleFilter::correct_pose_using_beam_range_finder_model(carmen_laser_laser_message *laser_message)
{
	double particle_weight_in_abscense_of_map = 1 / ((double) particles.size());

	//
	// if the first map has already been received, the particle weight is calculated in the
	// conventional way, using the beam_range_finder_model. if not, however, all the particles
	// receive the same weight
	//

	if (first_map_has_been_received)
	{
		unsigned int num_particles_with_weight_equals_to_zero = 0;

		update_particles_map_to_new_position();

		for(unsigned int i = 0; i < particles.size(); i++)
		{
			particles[i].weight = calculate_particle_probability(particles[i], laser_message);

			if (particles[i].weight == 0)
				num_particles_with_weight_equals_to_zero++;
		}

		if (num_particles_with_weight_equals_to_zero == particles.size())
			exit(printf("All particles have weight equals to zero!\n"));

		normalize_particle_weights();
	}
	else
	{
		for(unsigned int i = 0; i < particles.size(); i++)
			particles[i].weight = particle_weight_in_abscense_of_map;
	}

	//resample_particles();
}


void
ParticleFilter::update_particles_map_to_new_position()
{
	for(unsigned int i = 0; i < particles.size(); i++)
	{
		particles[i].map.move(particles[i].last_pose, particles[i].pose);
		particles[i].last_pose = particles[i].pose;
	}
}


void
ParticleFilter::add_obstacles_to_particles_map(carmen_laser_laser_message *laser_message)
{
	double x, y, angle, radius;

	carmen_pose_3D_t point;
	memset(&point, 0, sizeof(point));

	angle = laser_message->config.start_angle;

	for(int i = 0; i < laser_message->num_readings; i += 10)
	{
		angle = laser_message->config.start_angle + i * laser_message->config.angular_resolution;
		radius = laser_message->range[i];

		transform_polar_coordinates_to_cartesian_coordinates(radius, angle, &x, &y);

		point.position.x = x;
		point.position.y = y;
		point.orientation.yaw = angle;

		for(unsigned int j = 0; j < particles.size(); j++)
			particles[j].map.add(point, radius, angle, 0.0);
	}

	first_map_has_been_received = 1;
}


Particle
ParticleFilter::get_best_particle()
{
	int best_particle = 0;
	double max_weight = 0;

	for(unsigned int i = 0; i < particles.size(); i++)
	{
		if (particles[i].weight > max_weight)
		{
			max_weight = particles[i].weight;
			best_particle = i;
		}
	}

	return particles[best_particle];
}


vector<Particle>
ParticleFilter::get_particles()
{
	return particles;
}


double
ParticleFilter::get_distance_to_farthest_particle(Particle best_particle)
{
	double max_dist = 0, dist = 0;

	for(unsigned int i = 0; i < particles.size(); i++)
	{
		double abs_diff_x = pow(particles[i].pose.position.x - best_particle.pose.position.x, 2);
		double abs_diff_y = pow(particles[i].pose.position.y - best_particle.pose.position.y, 2);
		double abs_diff_z = pow(particles[i].pose.position.z - best_particle.pose.position.z, 2);

		dist = sqrt(abs_diff_x + abs_diff_y + abs_diff_z);

		if (dist > max_dist)
			max_dist = dist;
	}

	return max_dist;
}

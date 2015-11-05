/*
 * particle_filter.h
 *
 *  Created on: Dec 18, 2012
 *      Author: filipe mutz
 */

#ifndef __PARTICLE_FILTER_H_
#define __PARTICLE_FILTER_H_

#include <carmen/carmen.h>
#include <prob_motion_model.h>
#include "polar_map.h"

class Particle
{
	public:
		PolarMap map;
		double weight;
		carmen_pose_3D_t pose;
		carmen_pose_3D_t last_pose;

		Particle(int num_spheres, int num_angular_sections, int num_points_to_store);
		Particle(const Particle &p);
		~Particle();
};


class ParticleFilter
{
	int first_map_has_been_received;
	int _num_particles;
	vector<Particle> particles;

	void resample_particles();
	void normalize_particle_weights();
	double calculate_particle_probability(Particle &particle, carmen_laser_laser_message *laser_message);
	carmen_pose_3D_t generate_random_pose(carmen_pose_3D_t mean, carmen_pose_3D_t std);

	public:
		ParticleFilter();
		ParticleFilter(int num_particles, int num_spheres, int num_angular_sections, int num_points_to_store);
		~ParticleFilter();

		void generate_particles(carmen_pose_3D_t mean, carmen_pose_3D_t std);
		void predict_pose_using_motion_model(OdometryMotionCommand *ut);
		void correct_pose_using_beam_range_finder_model(carmen_laser_laser_message *laser_message);
		void add_obstacles_to_particles_map(carmen_laser_laser_message *laser_message);
		void update_particles_map_to_new_position();

		//
		// as funcoes abaixo foram designadas para debug e por isso sao
		// super lentas... nao use a nao ser que seja para fazer debug!
		//

		Particle get_best_particle();
		vector<Particle> get_particles();
		double get_distance_to_farthest_particle(Particle best_particle);
};

#endif /* __PARTICLE_FILTER_H_ */

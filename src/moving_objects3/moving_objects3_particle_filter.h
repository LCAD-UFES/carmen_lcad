#include <carmen/carmen.h>
#include <vector>

#include "moving_objects3_messages.h"
#include "moving_objects3_utils.h"
#include "polar_point.h"
#include "math.h"

#ifndef _MOVING_OBJECTS3_PARTICLE_FILTER_H_
#define _MOVING_OBJECTS3_PARTICLE_FILTER_H_

#define NUM_OF_PARTICLES 100
#define MAX_ACCELERATION 4.0
#define MAX_ANGULAR_VELOCITY 0.35

std::vector<moving_objects3_particle_t>
algorithm_particle_filter(std::vector<moving_objects3_particle_t> particle_set_t_1,
		double *virtual_scan, int num_of_rays,
		double delta_time);

std::vector<moving_objects3_particle_t>
scaling_series_particle_filter(std::vector<moving_objects3_particle_t> particle_set_t_1,
		double *virtual_scan, int num_of_rays,
		double delta_time);

std::vector<moving_objects3_particle_t>
importance_sampling(double *virtual_scan, int num_of_rays, int index, int num_of_particles);

#endif

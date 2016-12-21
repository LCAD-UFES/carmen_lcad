#include <carmen/carmen.h>
#include <vector>

#include "moving_objects3_messages.h"
#include "moving_objects3_utils.h"
#include "polar_point.h"
#include "math.h"

#ifndef _MOVING_OBJECTS3_PARTICLE_FILTER_H_
#define _MOVING_OBJECTS3_PARTICLE_FILTER_H_

#define NUM_OF_PARTICLES 25
#define MAX_ACCELERATION 2.0
#define MAX_ANGULAR_VELOCITY 0.35

std::vector<moving_objects3_particle_t>
algorithm_particle_filter(std::vector<moving_objects3_particle_t> particle_set_t_1,
		carmen_velodyne_projected_on_ground_message velodyne_projected_on_ground,
		double delta_time);

#endif

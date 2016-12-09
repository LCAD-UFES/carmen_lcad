#include <carmen/carmen.h>

#include <vector>
#include "math.h"

#ifndef _MOVING_OBJECTS3_PARTICLE_FILTER_H_
#define _MOVING_OBJECTS3_PARTICLE_FILTER_H_

#define NUM_OF_PARTICLES 100
#define MAX_ACCELERATION 1.0
#define MAX_ANGULAR_VELOCITY 0.25

typedef struct
{
	double width;	// object width
	double length;	// object length
	double c_x;		// anchor point x
	double c_y;		// anchor point y
} geometric_parameters;

typedef struct
{
	carmen_point_t pose;	// object pose (x, y, theta)
	double velocity;		// object velocity
	geometric_parameters geometry; // geomeric parameters
	double weight;			// particle weight
} moving_objects3_particle_t;

#endif

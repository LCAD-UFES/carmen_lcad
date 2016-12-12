/*
 * moving_objects3_utils.h
 *
 *  Created on: 12 de dez de 2016
 *      Author: luan
 */

#include <carmen/carmen.h>
#include "moving_objects3_particle_filter.h"

#ifndef SRC_MOVING_OBJECTS3_MOVING_OBJECTS3_UTILS_H_
#define SRC_MOVING_OBJECTS3_MOVING_OBJECTS3_UTILS_H_

typedef struct
{
	carmen_vector_2D_t p1;
	carmen_vector_2D_t p2;
	carmen_vector_2D_t p3;
	carmen_vector_2D_t p4;
} rectangle_points;

void generate_rectangles_points(moving_objects3_particle_t particle_t, rectangle_points* r1, rectangle_points* r2, rectangle_points* r3);

#endif /* SRC_MOVING_OBJECTS3_MOVING_OBJECTS3_UTILS_H_ */

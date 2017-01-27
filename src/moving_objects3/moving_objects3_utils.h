/*
 * moving_objects3_utils.h
 *
 *  Created on: 12 de dez de 2016
 *      Author: luan
 */

#include <carmen/carmen.h>

#ifndef SRC_MOVING_OBJECTS3_MOVING_OBJECTS3_UTILS_H_
#define SRC_MOVING_OBJECTS3_MOVING_OBJECTS3_UTILS_H_

typedef struct
{
	carmen_vector_2D_t p1;
	carmen_vector_2D_t p2;
	carmen_vector_2D_t p3;
	carmen_vector_2D_t p4;
} rectangle_points;

rectangle_points
generate_rectangle(double width, double length);

rectangle_points
transform_rectangle(rectangle_points rect, double x, double y, double theta);

void
generate_rectangles_points(carmen_point_t pose, double width, double length,
		rectangle_points* r1, rectangle_points* r2, rectangle_points* r3,
		double surface_width);

int
check_ray_intersection(carmen_vector_2D_t end_point, rectangle_points rect);

double
euclidean_distance(double x1, double y1, double x2, double y2);

#endif /* SRC_MOVING_OBJECTS3_MOVING_OBJECTS3_UTILS_H_ */

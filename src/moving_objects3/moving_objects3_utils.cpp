/*
 * moving_objects3_utils.cpp
 *
 *  Created on: 12 de dez de 2016
 *      Author: luan
 */

#include "moving_objects3_utils.h"

carmen_vector_2D_t
rotate_point(carmen_vector_2D_t point, double theta)
{
	carmen_vector_2D_t point_rotated;

	point_rotated.x = point.x * cos(theta) - point.y * sin(theta);
	point_rotated.y = point.x * sin(theta) + point.y * cos(theta);

	return point_rotated;
}


rectangle_points
generate_rectangle(double width, double length)
{
	rectangle_points temp;

	temp.p1.x = length/2;
	temp.p1.y = width/2;

	temp.p2.x = length/2;
	temp.p2.y = -width/2;

	temp.p3.x = -length/2;
	temp.p3.y = -width/2;

	temp.p4.x = -length/2;
	temp.p4.y = width/2;

	return temp;
}


rectangle_points
transform_rectangle(rectangle_points rect, double x, double y, double theta)
{
	rectangle_points rect_transformed, tmp;

	tmp = rect;
	tmp.p1 = rotate_point(rect.p1, theta);
	tmp.p2 = rotate_point(rect.p2, theta);
	tmp.p3 = rotate_point(rect.p3, theta);
	tmp.p4 = rotate_point(rect.p4, theta);

	rect_transformed.p1.x = tmp.p1.x + x;
	rect_transformed.p2.x = tmp.p2.x + x;
	rect_transformed.p3.x = tmp.p3.x + x;
	rect_transformed.p4.x = tmp.p4.x + x;

	rect_transformed.p1.y = tmp.p1.y + y;
	rect_transformed.p2.y = tmp.p2.y + y;
	rect_transformed.p3.y = tmp.p3.y + y;
	rect_transformed.p4.y = tmp.p4.y + y;

	return rect_transformed;
}


void
generate_rectangles_points(moving_objects3_particle_t particle, rectangle_points* r1, rectangle_points* r2, rectangle_points* r3)
{
	rectangle_points temp;

	// generate inside car rectangle
	temp = generate_rectangle(particle.geometry.width, particle.geometry.length);
	*r1 = transform_rectangle(temp, particle.pose.x, particle.pose.y, particle.pose.theta);

	// generate car surface rectangle
	temp = generate_rectangle(particle.geometry.width + 0.2, particle.geometry.length + 0.2);
	*r2 = transform_rectangle(temp, particle.pose.x, particle.pose.y, particle.pose.theta);

	// generate car surface rectangle
	temp = generate_rectangle(particle.geometry.width + 1.2, particle.geometry.length + 1.2);
	*r3 = transform_rectangle(temp, particle.pose.x, particle.pose.y, particle.pose.theta);
}

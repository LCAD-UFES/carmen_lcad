/**
 * polar_point.h
 *
 *  Created on: Nov 27, 2012
 *      Author: fmutz
 */

#ifndef __CARMEN_POLAR_POINT_H__
#define __CARMEN_POLAR_POINT_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	double angle;
	double radius;

}carmen_polar_point_t;

void transform_polar_coordinates_to_cartesian_coordinates(double radius, double angle, double *x, double *y);
void transform_cartesian_coordinates_to_polar_coordinates(double x, double y, double *radius, double *angle);

#ifdef __cplusplus
}
#endif

#endif /* __CARMEN_POLAR_POINT_H__ */

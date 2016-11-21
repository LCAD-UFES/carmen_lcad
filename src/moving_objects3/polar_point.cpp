#include "polar_point.h"
#include <math.h>

void
transform_polar_coordinates_to_cartesian_coordinates(double radius, double angle, double *x, double *y)
{
	*x = radius * cos(angle);
	*y = radius * sin(angle);
}


void
transform_cartesian_coordinates_to_polar_coordinates(double x, double y, double *radius, double *angle)
{
	*angle = atan2(y, x);
	*radius = sqrt(pow(x, 2) + pow(y, 2)); // x / cos(*angle);
}


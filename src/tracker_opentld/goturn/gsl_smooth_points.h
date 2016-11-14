/*
gcc -Wall -I/usr/local/include -c smooth.c
gcc -L/usr/local/lib smooth.o -lgsl -lgslcblas -lm
./a.out
*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_math.h>
#include <vector>

typedef struct
{
	double x[2];
	double y[2];
	int size;
} Traj;

std::vector<carmen_ackerman_traj_point_t>
smooth_points(std::vector<carmen_ackerman_traj_point_t> points);

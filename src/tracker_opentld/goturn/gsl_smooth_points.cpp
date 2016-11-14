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

#include <carmen/carmen.h>
#include "gsl_smooth_points.h"

using namespace std;


double
my_f(const gsl_vector *v, void *params)
{
	Traj *p = (Traj *)params;
	int i, j, size = p->size - 2;
	double a=0, b=0, sum=0;
	
	double x_prev = p->x[0];			//x(i-1)
	double x      = gsl_vector_get(v, 0);		//x(i)
	double x_next = gsl_vector_get(v, 1);		//x(i+1)
	
	double y_prev = p->y[0];
	double y      = gsl_vector_get(v, size);
	double y_next = gsl_vector_get(v, size+1);
	
	//printf ("%f\n", y_prev);
	//printf ("%f\n", y);
	//printf ("%f\n", y_next);
	
	for (i = 2, j = (size+2); i < size; i++, j++)
	{
		a = x_next - (2*x) + x_prev;
		b = y_next - (2*y) + y_prev;
		sum += (a*a + b*b);
		//printf ("%f\n", x_next);
		
		x_prev = x;
		x      = x_next;
		x_next = gsl_vector_get(v, i);
		
		y_prev = y;
		y      = y_next;
		y_next = gsl_vector_get(v, j);
	}
	
	x_prev = x;
	x      = x_next;
	x_next = p->x[1];
	
	y_prev = y;
	y      = y_next;
	y_next = p->y[1];
	//printf ("%f\n", y_next);
	
	a = x_next - (2*x) + x_prev;
	b = y_next - (2*y) + y_prev;
	sum += (a*a + b*b);
	
	//printf("%%  %f\n", sum);
	
	return (sum);
}


// The gradient of f, df = (df/dx, df/dy)
void 
my_df (const gsl_vector *v, void *params, gsl_vector *df)
{
	Traj *p = (Traj *)params;
	int i, j, size = p->size - 2;
	
	double x_prev2= 0;
	double x_prev = p->x[0];
	double x      = gsl_vector_get(v, 0);
	double x_next = gsl_vector_get(v, 1);
	double x_next2= gsl_vector_get(v, 2);
	double sum_x  =  (10*x) - (8*x_next) + (2*x_next2) - (4*x_prev);
	gsl_vector_set(df, 0, sum_x);
	//printf ("%f\n", sum_x);
	
	double y_prev2= 0;
	double y_prev = p->y[0];
	double y      = gsl_vector_get(v, size);
	double y_next = gsl_vector_get(v, size+1);
	double y_next2= gsl_vector_get(v, size+2);
	double sum_y  = (10*y) - (8*y_next) + (2*y_next2) - (4*y_prev);
	gsl_vector_set(df, size, sum_y);
	//printf ("%f\n", sum_y);
	//printf ("%f\n", y_prev);
	//printf ("%f\n", y);
	//printf ("%f\n", y_next);
	//printf ("%f\n", y_next2);
	
	for (i = 3, j = (size+3); i < size; i++, j++)
	{
		x_prev2= x_prev;
		x_prev = x;
		x      = x_next;
		x_next = x_next2;
		x_next2= gsl_vector_get(v, i);
		sum_x = (2*x_prev2) - (8*x_prev) + (12*x) - (8*x_next) + (2*x_next2);
		gsl_vector_set(df, (i-2), sum_x);
		//printf ("%f\n", sum_x);
		
		y_prev2= y_prev;
		y_prev = y;
		y      = y_next;
		y_next = y_next2;
		y_next2= gsl_vector_get(v, j);
		sum_y = (2*y_prev2) - (8*y_prev) + (12*y) - (8*y_next) + (2*y_next2);
		gsl_vector_set(df, (j-2), sum_y);
		//printf ("%f\n", sum_y);
	}
	
	x_prev2= x_prev;
	x_prev = x;
	x      = x_next;
	x_next = x_next2;
	x_next2= p->x[1];
	sum_x  = (2*x_prev2) - (8*x_prev) + (12*x) - (8*x_next) + (2*x_next2);
	gsl_vector_set(df, size-2, sum_x);
	//printf ("%f\n", sum_x);
	
	y_prev2= y_prev; 
	y_prev = y;
	y      = y_next;
	y_next = y_next2;
	y_next2= p->y[1];
	sum_y  = (2*y_prev2) - (8*y_prev) + (12*y) - (8*y_next) + (2*y_next2);
	gsl_vector_set(df, (2*size)-2, sum_y);
	//printf ("%f\n", sum_y);
	
	x_prev2= x_prev;
	x_prev = x;
	x      = x_next;
	x_next = x_next2;
	sum_x  = (2*x_prev2) - (8*x_prev) + (10*x) - (4*x_next);
	gsl_vector_set(df, size-1, sum_x);
	//printf ("%f\n", sum_x);
	
	y_prev2= y_prev; 
	y_prev = y;
	y      = y_next;
	y_next = y_next2;
	sum_y  = (2*y_prev2) - (8*y_prev) + (10*y) - (4*y_next);
	gsl_vector_set(df, (2*size)-1, sum_y);
	//printf("%f\n", sum_y);
}


// Compute both f and df together
void 
my_fdf (const gsl_vector *x, void *params, double *f, gsl_vector *df) 
{
	*f = my_f(x, params); 
	my_df(x, params, df);
}


vector<carmen_ackerman_traj_point_t>
smooth_points(vector<carmen_ackerman_traj_point_t> points)
{
	size_t iter = 0;
	int status, i, j;
	vector<carmen_ackerman_traj_point_t> points_smoothed;

	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;

	gsl_vector *v;
	gsl_multimin_function_fdf my_func;
	
	Traj path;
	path.size = points.size();
	path.x[0] = points[0].x;
	path.x[1] = points.back().x;
	path.y[0] = points[0].y;
	path.y[1] = points.back().y;
	

	my_func.n = (2*path.size)-4;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = &path;

	v = gsl_vector_alloc ((2*path.size)-4);
	
	for (i=0, j=(path.size-2); i < (path.size-2); i++, j++)
	{
		gsl_vector_set (v, i, points[i+1].x);
		gsl_vector_set (v, j, points[i+1].y);
	}
	
	T = gsl_multimin_fdfminimizer_conjugate_fr;
	s = gsl_multimin_fdfminimizer_alloc (T, (2*path.size)-4);

	gsl_multimin_fdfminimizer_set (s, &my_func, v, 0.1, 0.0001);  //(function_fdf, gsl_vector, step_size, tol)
	
	do
	{
		iter++;
		status = gsl_multimin_fdfminimizer_iterate (s);

		if (status){
			printf("%%Saiu STATUS %d\n", status);
			break;
		}

		status = gsl_multimin_test_gradient (s->gradient, 0.01);        //(gsl_vector, epsabs) and  |g| < epsabs

		if (status == GSL_SUCCESS)
		{
			printf ("%%Minimum found!!!\n");
		}
	}
	while (status == GSL_CONTINUE && iter < 9999);

	carmen_ackerman_traj_point_t p;
	p.x = path.x[0];
	p.y = path.y[0];
	points_smoothed.push_back(p);

//	printf ("%%ITER %d\n", (int)iter);
//	printf ("a = [\n");
//	printf ("%f %f\n", path.x[0], path.y[0]);
	for (i=0, j=(path.size-2); i < (path.size-2); i++, j++)
	{
//		printf ("%f %f\n", gsl_vector_get (s->x, i), gsl_vector_get (s->x, j));
		p.x = gsl_vector_get (s->x, i);
		p.y = gsl_vector_get (s->x, j);
		points_smoothed.push_back(p);

	}
//	printf ("%f %f\n", path.x[1], path.y[1]);
//	printf ("];\nx = a(:,1);\ny = a(:,2);\nplot (x,y)");

	p.x = path.x[1];
	p.y = path.y[1];
	points_smoothed.push_back(p);
	
	gsl_multimin_fdfminimizer_free (s);
	gsl_vector_free (v);

	return points_smoothed;
}


#include "rddf_predict_optimizer.h"

#define MAX_ERROR 0.00001
#define XTOTAL 30

void
get_optimization_params(SplineControlParams *spc, ObjectiveFunctionParams &params, carmen_behavior_selector_road_profile_message *rddf_poses)
{
	params.spline_control_params = spc;
	params.n_poses = rddf_poses->number_of_poses;
	params.rddf_poses = rddf_poses->poses;
	params.plan_cost = 0.0;
	params.xtotal = XTOTAL;
}

double
euclidean_distance_optimizer (double x1, double x2, double y1, double y2)
{
	return ( sqrt(pow(x2-x1,2) + pow(y2-y1,2)) );
}

double
compute_distance_from_spline_to_rddf(SplineControlParams spc, carmen_ackerman_traj_point_t *rddf_poses)
{
	// constroi spline
	gsl_interp_accel *acc;
	gsl_spline *phi_spline;
	double knots_x[4] = {0.0,  spc.xtotal/ 3.0, 2 * spc.xtotal / 3.0, spc.xtotal};
	double knots_y[4] = {0.0, spc.k1, spc.k2, spc.k3};
	acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	phi_spline = gsl_spline_alloc(type, 4);
	gsl_spline_init(phi_spline, knots_x, knots_y, 4);
	int i = 0;
	double total_distance = 0;
	double spline_y;
	while (rddf_poses[i].x <=30)
	{
		double spline_x = rddf_poses[i].x;
		//Foi colocado essa condição pois o erro acontecia quando spline_x<0
		if(spline_x>0){
			spline_y = gsl_spline_eval(phi_spline, spline_x, acc);
			double distance = euclidean_distance_optimizer(rddf_poses[i].x, spline_x, rddf_poses[i].y, spline_y);
			total_distance += distance*distance;
		}
		i++;
	}

	return total_distance/i;
}

SplineControlParams
fill_in_spline_params(const gsl_vector *x, ObjectiveFunctionParams *my_params)
{
	SplineControlParams spline_control_params;

	spline_control_params.k1 = gsl_vector_get(x, 0);
	spline_control_params.k2 = gsl_vector_get(x, 1);
	spline_control_params.k3 = gsl_vector_get(x, 2);
	spline_control_params.xtotal = my_params->xtotal;

	return spline_control_params;
}

double
my_f(const gsl_vector *x, void *params)
{
	double w1 = 1.0;

	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;

	SplineControlParams spline_control_params = fill_in_spline_params(x, my_params);

	double distance_result = compute_distance_from_spline_to_rddf(spline_control_params, my_params->rddf_poses);

	double result = sqrt(w1*distance_result);
	return result;
}

void
my_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	double h;

	double f_x = my_f(v, params);

	h = 0.0002;

	gsl_vector *x_h;
	x_h = gsl_vector_alloc(3);

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0) + h);
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	double f_x_h = my_f(x_h, params);
	double d_f_x_h = (f_x_h - f_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1) + h);
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	double f_y_h = my_f(x_h, params);
	double d_f_y_h = (f_y_h - f_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2) + h);
	double f_z_h = my_f(x_h, params);
	double d_f_z_h = (f_z_h - f_x) / h;

	gsl_vector_set(df, 0, d_f_x_h);
	gsl_vector_set(df, 1, d_f_y_h);
	gsl_vector_set(df, 2, d_f_z_h);

	gsl_vector_free(x_h);
}


void
my_fdf(const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f(x, params);
	my_df(x, params, df);
}
/*
double interpolate(carmen_position_t p1, carmen_position_t p2, double target_x)
{
	return p1.y + (target_x - p1.x) * (p2.y - p1.y) / (p2.x - p1.x);
}

SplineControlParams
get_seed_knots(carmen_behavior_selector_road_profile_message rddf_poses, double xtotal)
{
	SplineControlParams knots_seed;
	knots_seed.xtotal = xtotal;
	knots_seed.valid = true;

	double distance, min_distance1, min_distance2 = DBL_MAX;
	carmen_position_t nearest1, nearest2;
	nearest1.x, nearest2.x, nearest1.y, nearest2.y = DBL_MAX;
	for (int i = 0; i < rddf_poses.number_of_poses; i++)
	{
		distance = abs(rddf_poses.poses[i].x - xtotal/3.0);
		if (distance < min_distance1 && distance < min_distance2)
		{
			if (min_distance1 < min_distance2)
			{
				if (rddf_poses.poses[i].x != nearest1.x)
				{
					min_distance2 = distance;
					nearest2.x = rddf_poses.poses[i].x;
					nearest2.y = rddf_poses.poses[i].y;
				}
			}
			else
			{
				if (rddf_poses.poses[i].x != nearest2.x)
				{
					min_distance1 = distance;
					nearest1.x = rddf_poses.poses[i].x;
					nearest1.y = rddf_poses.poses[i].y;
				}
			}
		}
		else if (distance < min_distance1 || distance < min_distance2)
		{
			if (distance < min_distance2)
			{
				if (rddf_poses.poses[i].x != nearest1.x)
				{
					min_distance2 = distance;
					nearest2.x = rddf_poses.poses[i].x;
					nearest2.y = rddf_poses.poses[i].y;
				}
			}
			else
			{
				if (rddf_poses.poses[i].x != nearest2.x)
				{
					min_distance1 = distance;
					nearest1.x = rddf_poses.poses[i].x;
					nearest1.y = rddf_poses.poses[i].y;
				}
			}
		}
	}

	knots_seed.k1 = interpolate(nearest1, nearest2, xtotal/3.0);

	distance, min_distance1, min_distance2 = DBL_MAX;
	nearest1.x, nearest2.x, nearest1.y, nearest2.y = DBL_MAX;
	for (int i = 0; i < rddf_poses.number_of_poses; i++)
	{
		distance = abs(rddf_poses.poses[i].x - 2*xtotal/3.0);
		if (distance < min_distance1 && distance < min_distance2)
		{
			if (min_distance1 < min_distance2)
			{
				if (rddf_poses.poses[i].x != nearest1.x)
				{
					min_distance2 = distance;
					nearest2.x = rddf_poses.poses[i].x;
					nearest2.y = rddf_poses.poses[i].y;
				}
			}
			else
			{
				if (rddf_poses.poses[i].x != nearest2.x)
				{
					min_distance1 = distance;
					nearest1.x = rddf_poses.poses[i].x;
					nearest1.y = rddf_poses.poses[i].y;
				}
			}
		}
		else if (distance < min_distance1 || distance < min_distance2)
		{
			if (distance < min_distance2)
			{
				if (rddf_poses.poses[i].x != nearest1.x)
				{
					min_distance2 = distance;
					nearest2.x = rddf_poses.poses[i].x;
					nearest2.y = rddf_poses.poses[i].y;
				}
			}
			else
			{
				if (rddf_poses.poses[i].x != nearest2.x)
				{
					min_distance1 = distance;
					nearest1.x = rddf_poses.poses[i].x;
					nearest1.y = rddf_poses.poses[i].y;
				}
			}
		}
	}

	knots_seed.k2 = interpolate(nearest1, nearest2, 2*xtotal/3.0);

	distance, min_distance1, min_distance2 = DBL_MAX;
	nearest1.x, nearest2.x, nearest1.y, nearest2.y = DBL_MAX;
	for (int i = 0; i < rddf_poses.number_of_poses; i++)
	{
		distance = abs(rddf_poses.poses[i].x - xtotal);
		if (distance < min_distance1 && distance < min_distance2)
		{
			if (min_distance1 < min_distance2)
			{
				if (rddf_poses.poses[i].x != nearest1.x)
				{
					min_distance2 = distance;
					nearest2.x = rddf_poses.poses[i].x;
					nearest2.y = rddf_poses.poses[i].y;
				}
			}
			else
			{
				if (rddf_poses.poses[i].x != nearest2.x)
				{
					min_distance1 = distance;
					nearest1.x = rddf_poses.poses[i].x;
					nearest1.y = rddf_poses.poses[i].y;
				}
			}
		}
		else if (distance < min_distance1 || distance < min_distance2)
		{
			if (distance < min_distance2)
			{
				if (rddf_poses.poses[i].x != nearest1.x)
				{
					min_distance2 = distance;
					nearest2.x = rddf_poses.poses[i].x;
					nearest2.y = rddf_poses.poses[i].y;
				}
			}
			else
			{
				if (rddf_poses.poses[i].x != nearest2.x)
				{
					min_distance1 = distance;
					nearest1.x = rddf_poses.poses[i].x;
					nearest1.y = rddf_poses.poses[i].y;
				}
			}
		}
	}

	knots_seed.k3 = interpolate(nearest1, nearest2, xtotal);

	return knots_seed;
}
*/
/*
SplineControlParams
optimize_spline_knots(carmen_behavior_selector_road_profile_message *last_rddf_poses)
{
	SplineControlParams knots_seed = get_seed_knots(*last_rddf_poses, 30.0);
	ObjectiveFunctionParams params;
	get_optimization_params(&knots_seed, params, last_rddf_poses);

	gsl_multimin_function_fdf my_func;

	int n_optimized_knots = 3;
	my_func.n = n_optimized_knots;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = &params;

	// Starting point, x
	gsl_vector *x = gsl_vector_alloc(n_optimized_knots);
	gsl_vector_set(x, 0, knots_seed.k1);
	gsl_vector_set(x, 1, knots_seed.k2);
	gsl_vector_set(x, 2, knots_seed.k3);

	const gsl_multimin_fdfminimizer_type *T = gsl_multimin_fdfminimizer_conjugate_fr;
	gsl_multimin_fdfminimizer *s = gsl_multimin_fdfminimizer_alloc(T, n_optimized_knots);

	// int gsl_multimin_fdfminimizer_set (gsl_multimin_fdfminimizer * s, gsl_multimin_function_fdf * fdf, const gsl_vector * x, double step_size, double tol)
	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.005, 0.1);

	size_t iter = 0;
	int status;
	do
	{
		iter++;

		status = gsl_multimin_fdfminimizer_iterate(s);

		if (status == GSL_ENOPROG) // minimizer is unable to improve on its current estimate, either due to numerical difficulty or a genuine local minimum
			break;

		status = gsl_multimin_test_gradient(s->gradient, 0.16); // esta funcao retorna GSL_CONTINUE ou zero

	} while ((s->f > MAX_ERROR) && (status == GSL_CONTINUE) && (iter < 50));

	SplineControlParams spline_params_result = fill_in_spline_params(s->x, &params);

	printf("iter = %d\n", iter);

	if ((s->f > MAX_ERROR) && (iter > 50))
	{
		spline_params_result.valid = false;
	}

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (spline_params_result);
}
*/



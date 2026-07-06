#include <stdio.h>
#include <list>
#include <carmen/carmen.h>
#include <carmen/collision_detection.h>
#include <gsl/gsl_multimin.h>

#include "nlp_mat_planner.h"

#define DIFF2D(x1,x2) {x1.x - x2.x, x1.y - x2.y, 0.0, 0, {0.0, 0.0, 0.0, 0.0, 0.0}, 0.0, 0.0}


double w1, w2, w3, w4, w5;

typedef struct
{
	carmen_robot_and_trailers_traj_point_t *input_path;
	int input_size;
	int problem_size;
	bool *anchor_point;
	carmen_obstacle_distance_mapper_map_message *obstacle_distance_map;
	double dmin;
	double kmax;
	bool plot_costs;
	//Funcao de custo de reta entre pontos ancoras
	int *between_anchor_points;
	int between_anchor_points_size;
	carmen_robot_and_trailers_traj_point_t *anchor_points_list;

} params_t;


using namespace std;

extern double distance_between_front_and_rear_axles;

FILE *gnuplot_pipe_costs;


void
plot_costs(double roughness, double curviness, double sum_distance_to_nearest_obstacles, double displacement_from_anchor_points_thetas)
{
	static bool first_time = true;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipe_costs = popen("gnuplot", "w"); // -persist to keep last plot after program closes

		FILE *gnuplot_data_file = fopen("gnuplot_costs_data.txt", "w");
		fclose(gnuplot_data_file);
	}

	FILE *gnuplot_data_file = fopen("gnuplot_costs_data.txt", "a");
	fprintf(gnuplot_data_file, "%lf, %lf, %lf, %lf\n", roughness, curviness, sum_distance_to_nearest_obstacles, displacement_from_anchor_points_thetas);
	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe_costs, "plot "
			"'./gnuplot_costs_data.txt' u 1 w l lt rgb 'red'    t 'roughness',"
			"'./gnuplot_costs_data.txt' u 2 w l lt rgb 'green'  t 'curviness',"
			"'./gnuplot_costs_data.txt' u 3 w l lt rgb 'blue'   t 'sum_distance_to_nearest_obstacles',"
			"'./gnuplot_costs_data.txt' u 4 w l lt rgb 'purple' t 'displacement_from_anchor_points_thetas'\n");

	fflush(gnuplot_pipe_costs);
}


void
compute_phi(carmen_robot_and_trailers_traj_point_t *path, int num_poses)
{
	double L = distance_between_front_and_rear_axles;

	for (int i = 0; i < (num_poses - 1); i++)
	{
		double delta_theta = carmen_normalize_theta(path[i + 1].theta - path[i].theta);
		double l = DIST2D(path[i], path[i + 1]);
		if (l < 0.01)
		{
			path[i].phi = 0.0;
			continue;
		}
		path[i].phi = atan(L * (delta_theta / l));
	}

	for (int i = 1; i < (num_poses - 1); i++)
		path[i].phi = (path[i].phi + path[i - 1].phi + path[i + 1].phi) / 3.0;
}


void
compute_theta(carmen_robot_and_trailers_traj_point_t *path, int num_poses)
{
	for (int i = 0; i < (num_poses - 1); i++)
		if (path[i].v >= 0.0)
			path[i].theta = atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
		else
			path[i].theta = atan2(path[i].y - path[i + 1].y, path[i].x - path[i + 1].x);

//	if (num_poses > 1)
//		path[num_poses - 1].theta = path[num_poses - 2].theta;
}


void
compute_theta_and_phi(carmen_robot_and_trailers_traj_point_t *poses_ahead, int num_poses_ahead)
{
	compute_theta(poses_ahead, num_poses_ahead);
	compute_phi(poses_ahead, num_poses_ahead);
}


/////////////////////////////////////////////////////////////////////////////////

double
my_f(const gsl_vector *v, void *params)
{	// Dolgov, Dmitri & Thrun, Sebastian & Montemerlo, Michael & Diebel, James. (2010). Path Planning for Autonomous Vehicles
	// in Unknown Semi-structured Environments. I. J. Robotic Res.. 29. 485-501. Section 3.1.

	params_t *param = (params_t *) params;

	// atualiza o path com os valores atuais, sem mecher nos anchor points
	int j = 0;
	for (int i = 0; i < param->input_size; i++)
	{
		if (!param->anchor_point[i])
		{
			param->input_path[i].x = gsl_vector_get(v, j); j++;
			param->input_path[i].y = gsl_vector_get(v, j); j++;
			//Essa parte só serve para atualizar o theta de cada pose, para facilitar seu calculo de custo (especialmente quando o custo precisa ser afetado por um ponto ancora vizinho)
			if (param->input_path[i].v > 0.0)
				param->input_path[i].theta = atan2(param->input_path[i + 1].y - param->input_path[i].y, param->input_path[i + 1].x - param->input_path[i].x);
			else
				param->input_path[i].theta = atan2(param->input_path[i + 1].y - param->input_path[i].y, param->input_path[i + 1].x - param->input_path[i].x) + M_PI;
			
			
		}
	}
	// Valor 1.0 porque às vezes dá menos que 1, o que faz com que o custo diminua de acordo com o weight, em vez de aumentar proporcionalmente
	double roughness = 1.0;
	double curviness = 1.0;
	double line_following = 1.0;
	for (int i = 1; i < (param->input_size - 1); i++)
	{

		carmen_robot_and_trailers_traj_point_t delta_xi = DIFF2D(param->input_path[i], param->input_path[i - 1]);
		carmen_robot_and_trailers_traj_point_t delta_xi_p_1 = DIFF2D(param->input_path[i + 1], param->input_path[i]);


		carmen_robot_and_trailers_traj_point_t delta = DIFF2D(delta_xi_p_1, delta_xi);
		if (!param->anchor_point[i])
		{ 	
			roughness += DOT2D(delta, delta); // (delta_xi_p_1 - delta_xi)^2
		}

		if (!param->anchor_point[i] && param->between_anchor_points[i] != -1 )
		{
			line_following += (DIST2D(param->anchor_points_list[param->between_anchor_points[i]-1], param->input_path[i]) + DIST2D(param->input_path[i], param->anchor_points_list[param->between_anchor_points[i]])) ;
		}

		double delta_angle_ = acos(DOT2D(delta_xi, delta_xi_p_1) / (sqrt(DOT2D(delta_xi, delta_xi)) * sqrt(DOT2D(delta_xi_p_1, delta_xi_p_1))));
//		double delta_angle_ = carmen_normalize_theta(atan(delta_xi_p_1.y / delta_xi_p_1.x) - atan(delta_xi.y / delta_xi.x));
		double delta_angle = fabs(delta_angle_) / sqrt(DOT2D(delta_xi, delta_xi));
		if (!param->anchor_point[i])
			if ((delta_angle > param->kmax))
			{
				curviness += delta_angle - param->kmax;
			}
	}

	double sum_distance_to_nearest_obstacles = 1.0;
	for (int i = 0; i < param->input_size - 1; i++)
	{
		double distance = carmen_obstacle_avoider_car_distance_to_nearest_obstacle(param->input_path[i], param->obstacle_distance_map);
		
		if (!param->anchor_point[i] && (distance < param->dmin))
			sum_distance_to_nearest_obstacles += param->dmin - distance;

	}

	double displacement_from_anchor_points_thetas = 1.0;
	for (int i = 0; i < (param->input_size - 1); i++)
	{
		if (param->anchor_point[i])
		{
			double delta_theta;
			if (param->input_path[i].v < 0.0)
			{
				delta_theta = param->input_path[i].theta -
						atan2(param->input_path[i].y - param->input_path[i + 1].y,
							  param->input_path[i].x - param->input_path[i + 1].x);
			}
			else
			{
				delta_theta = param->input_path[i].theta -
						atan2(param->input_path[i + 1].y - param->input_path[i].y,
							  param->input_path[i + 1].x - param->input_path[i].x);
			}
			displacement_from_anchor_points_thetas += fabs(carmen_normalize_theta(delta_theta));
		}
	}

	int i = param->input_size - 1;
	double delta_theta_n;
	
	if (param->input_path[i].v >= 0.0)
		delta_theta_n = param->input_path[i].theta - atan2(param->input_path[i].y - param->input_path[i - 1].y, param->input_path[i].x - param->input_path[i - 1].x);
	else
		delta_theta_n = param->input_path[i].theta - atan2(param->input_path[i - 1].y - param->input_path[i].y, param->input_path[i - 1].x - param->input_path[i].x);

	displacement_from_anchor_points_thetas += fabs(carmen_normalize_theta(delta_theta_n));

//	printf("Roughness = %f\n");
	double f = w1 * roughness +
		   w2 * curviness * curviness +
		   w3 * sum_distance_to_nearest_obstacles * sum_distance_to_nearest_obstacles +
		   w4 * displacement_from_anchor_points_thetas * displacement_from_anchor_points_thetas +
		   w5 * line_following;

	if (param->plot_costs)
		plot_costs(roughness, curviness * curviness, sum_distance_to_nearest_obstacles * sum_distance_to_nearest_obstacles,
				displacement_from_anchor_points_thetas * displacement_from_anchor_points_thetas);

	return (f);
}


void
my_df (const gsl_vector *v, void *params, gsl_vector *df)
{
	double h = 0.00005;
	double f_x = my_f(v, params);

	params_t *param = (params_t *) params;

	gsl_vector *x_h = gsl_vector_alloc(param->problem_size);
	gsl_vector_memcpy(x_h, v);

	int j = 0;
	for (int i = 0; i < param->input_size; i++)
	{
		if (!param->anchor_point[i])
		{
			// x
			gsl_vector_set(x_h, j, gsl_vector_get(v, j) + h);
			double f_x_h = my_f(x_h, params);
			double d_f_x_h = (f_x_h - f_x) / h;
			gsl_vector_set(df, j, d_f_x_h);
			gsl_vector_set(x_h, j, gsl_vector_get(v, j));
			j++;

			// y
			gsl_vector_set(x_h, j, gsl_vector_get(v, j) + h);
			f_x_h = my_f(x_h, params);
			d_f_x_h = (f_x_h - f_x) / h;
			gsl_vector_set(df, j, d_f_x_h);
			gsl_vector_set(x_h, j, gsl_vector_get(v, j));
			j++;
		}
	}

	gsl_vector_free(x_h);
}


void
my_fdf (const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f(x, params);
	my_df(x, params, df);
}


static int
sign(double x)
{
	if (x >= 0.0)
		return (1);
	else
		return (-1);
}


bool
is_anchor_point(int i, carmen_robot_and_trailers_traj_point_t *input_path, int size)
{
	if ((i == 0) || (i == (size - 1)))
		return (true);

	if ((i >= 1) && (i < (size - 1)) &&
		(sign(input_path[i].v) != sign(input_path[i + 1].v)))
		return (true);

	if ((i >= 1) && (i < (size - 1)) &&
		((sign(input_path[i].v) != sign(input_path[i + 1].v)) ||
		 (sign(input_path[i].v) != sign(input_path[i - 1].v))))
		return (true);

	if ((i >= 1) && (i < (size - 2)) &&
		(sign(input_path[i].v) != sign(input_path[i + 2].v)))
		return (true);
//
//	if ((i >= 2) && (sign(input_path[i - 1].v) != sign(input_path[i - 2].v)))
//		return (true);

	return (false);
}


int
set_anchor_points(params_t &params, carmen_robot_and_trailers_traj_point_t *input_path)
{

	int bet_ap = 0;
	params.anchor_point[params.input_size - 1] = true;

	for (int i = 0; i < params.input_size; i++)
	{
		if (is_anchor_point(i, input_path, params.input_size))
		{
			params.anchor_point[i] = true;
			bet_ap++;
		}
		else
		{
			params.anchor_point[i] = false;
			params.between_anchor_points[i] = bet_ap;
		}
	}

	params.between_anchor_points_size = bet_ap+1; // +1 por causa do último ponto do path
	params.problem_size = 0;

	for (int i = 0; i < params.input_size; i++)
		if (!params.anchor_point[i])
			params.problem_size += 2;

	return (params.problem_size);
}


int
remove_small_reverse_moves(carmen_robot_and_trailers_traj_point_t *input_path, int num_poses)
{
	for (int i = 0; i < (num_poses - 2); i++)
	{
		if ((sign(input_path[i].v) == sign(input_path[i + 2].v)) && (sign(input_path[i].v) != sign(input_path[i + 1].v)))  
		{
			for (int j = i + 1; j < (num_poses - 1); j++)
				input_path[j] = input_path[j + 1];
			num_poses--;
		}
	}

	return (num_poses);
}


bool
smooth_path_using_conjugate_gradient_study(carmen_robot_and_trailers_traj_point_t *input_path, int &num_poses,
		carmen_obstacle_distance_mapper_map_message *obstacle_distance_map, double obstacles_safe_distance,
		double max_phi, double smoothness_cost, double curvature_cost, double obstacle_cost, double theta_displacement_cost, double line_following_cost)
{

	w1 = smoothness_cost;
	w2 = curvature_cost;
	w3 = obstacle_cost;
	w4 = theta_displacement_cost;
	w5 = line_following_cost;
	printf("\nWeight Costs: smoothness_cost = %lf curvature_cost = %lf obstacle_cost = %lf theta_displacement_cost = %lf line_following_cost = %lf\n", w1, w2, w3, w4, w5);

	if (w1+w2+w3+w4+w5 == 0.0)
	{
		return 1;
	}


	params_t params;
	params.input_path = input_path;

	params.input_size = remove_small_reverse_moves(input_path, num_poses);

	params.obstacle_distance_map = obstacle_distance_map;
	params.dmin = obstacles_safe_distance;
	params.kmax = max_phi/ distance_between_front_and_rear_axles;
	
	int *between_anchor_points = (int *) malloc(sizeof(int) * params.input_size);

	for (int i = 0; i < params.input_size; i++)
	{
		between_anchor_points[i] = -1;
	}

	params.between_anchor_points = between_anchor_points;

	params.anchor_point = (bool *) malloc(params.input_size * sizeof(bool));
	params.between_anchor_points = between_anchor_points;
	params.problem_size = set_anchor_points(params, input_path);

	params.plot_costs = false;

	params.anchor_points_list = (carmen_robot_and_trailers_traj_point_t*) malloc(sizeof(carmen_robot_and_trailers_traj_point_t)*(params.between_anchor_points_size));

	int current_anchor = 0;

	for (int i = 0; i < params.input_size; i++)
	{
		if (params.anchor_point[i])
		{
			params.anchor_points_list[current_anchor] = params.input_path[i];
			current_anchor++;
		}
	}
	
	gsl_multimin_function_fdf my_func;
	my_func.n = params.problem_size;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = &params;

	gsl_vector *v = gsl_vector_alloc(params.problem_size);

	int j = 0;
	for (int i = 0; i < params.input_size; i++)
	{
		if (!params.anchor_point[i])
		{
			gsl_vector_set(v, j, params.input_path[i].x); j++;
			gsl_vector_set(v, j, params.input_path[i].y); j++;
		}
	}

	if (j != params.problem_size)
	{
		printf("Error! j != params.problem_size in smooth_path_using_conjugate_gradient()\n");
		exit(1);
	}

	const gsl_multimin_fdfminimizer_type *T = gsl_multimin_fdfminimizer_conjugate_fr;
	gsl_multimin_fdfminimizer *s = gsl_multimin_fdfminimizer_alloc(T, params.problem_size);

	gsl_multimin_fdfminimizer_set(s, &my_func, v, 0.01, 0.0001);

	int iter = 0;
	int status;
	do
	{
		iter++;
		status = gsl_multimin_fdfminimizer_iterate(s);
		if ((status != GSL_SUCCESS) && (status != GSL_ENOPROG) && (status != GSL_CONTINUE))
		{
			printf("optimization failed... code %d, iter %d\n", status, iter);

			gsl_multimin_fdfminimizer_free (s);
			gsl_vector_free (v);
			free(params.anchor_point);

			return (false);
		}

//		params.plot_costs = true;
//		my_f(s->x, (void *) &params);
//		params.plot_costs = false;

	} while ((status != GSL_ENOPROG) && (iter < 250));

	compute_theta_and_phi(input_path, params.input_size);
	printf("path smooth: iter %d, status %d, cost %lf, removed back moves %d\n", iter, status, s->f, num_poses - params.input_size);
	num_poses = params.input_size;

	gsl_multimin_fdfminimizer_free (s);
	gsl_vector_free (v);
	free(params.anchor_point);
	free(params.between_anchor_points);
	free(params.anchor_points_list);


	return (true);
}

//////////////////////////////////////////////////////////////////////////////////


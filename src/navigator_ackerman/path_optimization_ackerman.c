/*
 * path_optimization_ackerman.c
 *
 *  Created on: Jan 16, 2013
 *      Author: mgoncalves
 */
#include <carmen/carmen.h>
#include <assert.h>
#include "planner_ackerman_interface.h"
#include "conventional_ackerman.h"
#include "conventional_astar_ackerman.h"
#include "navigator_ackerman.h"
#include "trajectory_ackerman.h"


extern carmen_robot_ackerman_config_t robot_conf_g;
extern carmen_navigator_ackerman_astar_t astar_config;

double Ws = 1;
double Wk = 1;


carmen_robot_and_trailers_traj_point_t
carmen_conventional_astar_ackerman_kinematic_optimization_2(carmen_robot_and_trailers_traj_point_t point, double lenght, double phi, double v)
{
	//int phi_signal = phi >= 0 ? 1 : -1;
	//phi = carmen_degrees_to_radians(phi);

	double interval_time = 1;
	double time, rest;
	int i;
	int	   n;
	time = 0.1;
	n = interval_time / time;
	rest = interval_time - (n * time);

	for (i = 0; i < n; i++)
	{
		point.x += v * time * cos(point.theta);
		point.y += v * time * sin(point.theta);
		point.theta += v * time * ((tan(point.phi)) / lenght);

	//	point.phi += atan(robot_conf_g.distance_between_front_and_rear_axles * robot_conf_g.maximum_steering_command_rate * fabs(v) * time) * phi_signal;

		/*
		if (phi != 0)
		{
			if (point.phi - phi < carmen_degrees_to_radians(5))
			{
				point.phi = phi;
			}
		}
		else
		{
			if (fabs(point.phi) < phi)
			{
				point.phi = phi;
			}

		}
*/
		point.phi = phi;
	}
	point.x += v * rest * cos(point.theta);
	point.y += v * rest * sin(point.theta);
	point.theta += v * rest * ((tan(point.phi)) / lenght);
	point.theta = carmen_normalize_theta(point.theta);

	point.v = v;

	return point;
}

carmen_robot_and_trailers_traj_point_t
carmen_conventional_astar_ackerman_kinematic_optimization(carmen_robot_and_trailers_traj_point_t point, double lenght, double phi, double v)
{
	//phi = carmen_degrees_to_radians(phi);
	point.theta += v * ((tan(phi)) / lenght);
	point.theta = carmen_normalize_theta(point.theta);
	point.x += v * cos(point.theta);
	point.y += v * sin(point.theta);
	point.v = v;
	point.phi = phi;
	return point;
}



double delta_phi(carmen_robot_and_trailers_traj_point_t x1, carmen_robot_and_trailers_traj_point_t x2, carmen_robot_and_trailers_traj_point_t x3)
{
	double delta_phi;
	delta_phi = fabs(1 / tan((x3.y - x2.y) / (x3.x - x2.x)) - 1 / tan((x2.y - x1.y) / (x2.x - x1.x)));
	return delta_phi;

}


carmen_robot_and_trailers_traj_point_t delta_X(carmen_robot_and_trailers_traj_point_t x1, carmen_robot_and_trailers_traj_point_t x2)
{
	carmen_robot_and_trailers_traj_point_t delta_X;
	delta_X.x = (x1.x - x2.x);
	delta_X.y = (x1.y - x2.y);
	return delta_X;
}


double curvature_measure(carmen_robot_and_trailers_traj_point_t *points, int length)
{
	int index;
	carmen_robot_and_trailers_traj_point_t X;
	double soma = 0;
	for (index = 1; index < length - 1; index++)
	{
		X = delta_X(points[index], points[index - 1]);
		soma += pow(delta_phi(points[index - 1], points[index], points[index + 1]) / fabs(X.x + X.y), 2);
	}
	soma = Wk * soma;
	//printf("curvature_measure %f\t",soma);
	return soma;
}


double smoothness_measure(carmen_robot_and_trailers_traj_point_t *points, int length)
{
	int index;
	carmen_robot_and_trailers_traj_point_t X;
	double soma = 0;
	for (index = 1; index < length - 1; index++)
	{
		X = delta_X(delta_X(points[index + 1], points[index]), delta_X(points[index], points[index - 1]));
		soma += pow(X.x + X.y, 2);
	}
	soma = Ws * soma;
	//printf("smoothness_measure %f\n",soma);
	return soma;
}

double avaliation(carmen_robot_and_trailers_traj_point_t *points, int length)
{
	double result = 0;
	result = curvature_measure(points, length) + smoothness_measure(points, length);
	return result;
}


int RandomInteger( int low, int high)
{
    int k;
    double d;
    d = (double) rand( ) / ((double) RAND_MAX + 1);
    k = d * (high - low + 1);
    return low + k;
}


void pertubation(carmen_robot_and_trailers_traj_point_t *points, int length)
{
	int index;
	for (index = 0; index < length; index++)
	{
		//printf("INICIO\tindex: %d\tphi: %.2f\t theta: %.2f\n",index, carmen_radians_to_degrees(path.points[index].phi), carmen_radians_to_degrees(path.points[index].theta));

	}

	index = RandomInteger(1, length);
	points[index].phi -= carmen_degrees_to_radians(2);
	//printf("index 1= %d\n",index);

	index = RandomInteger(1, length);
	points[index].phi += carmen_degrees_to_radians(2);
	//printf("index 2= %d\n",index);


	for (index = 1; index < length; index++)
	{
		points[index] = carmen_conventional_astar_ackerman_kinematic_optimization(points[index-1], robot_conf_g.distance_between_front_and_rear_axles, points[index].phi, astar_config.path_interval);
	}
	//printf("index = %d\n",index);
	for (index = 0; index < length; index++)
	{
		//printf("INTERNO\tindex: %d\tphi: %.2f\t theta: %.2f\n",index, carmen_radians_to_degrees(path.points[index].phi), carmen_radians_to_degrees(path.points[index].theta));

	}
}

void copy_points(carmen_robot_and_trailers_traj_point_t *points1, carmen_robot_and_trailers_traj_point_t *points2, int length)
{
	int index;
	for (index = 0; index < length; index++)
	{
		points2[index] = points1[index];
	}
}


void carmen_path_optimization(carmen_robot_and_trailers_traj_point_t start, carmen_robot_and_trailers_traj_point_t goal,carmen_planner_path_p path)
{

	if (start.phi && goal.v)
	{
		//tirando warning
	}

	double current_avaliation = 0;
	int length = path->length;
	double best_avaliation = 9999999999999999;
	carmen_robot_and_trailers_traj_point_t last_point = path->points[length-1];
	carmen_robot_and_trailers_traj_point_t *best_points;
	carmen_robot_and_trailers_traj_point_t *new_points;
	best_points = calloc(length, sizeof(carmen_robot_and_trailers_traj_point_t));
	new_points = calloc(length, sizeof(carmen_robot_and_trailers_traj_point_t));


	//carmen_planner_path_t best_path = *path;
	//carmen_planner_path_t new_path;
	int index;
	copy_points(path->points, best_points,length);

	current_avaliation = avaliation(path->points, length);
	for (index = 0; index < 1000000; index++)
	{
		copy_points(best_points, new_points,length);
		pertubation(new_points, length);

		current_avaliation = avaliation(new_points, length);
		if (current_avaliation < best_avaliation)
		{
			if (fabs(last_point.theta) - fabs(new_points[length-1].theta) < carmen_degrees_to_radians(10))
			{
				best_avaliation = current_avaliation;
				copy_points(new_points, best_points,length);
			}

		}
	}
	printf("ANTIGO\n");
	for (index = 0; index < length; index++)
	{
		printf("index: %d\tphi: %.2f\t x: %.2f y %.2f theta: %.2f\n",index, carmen_radians_to_degrees(path->points[index].phi), path->points[index].x,path->points[index].y, carmen_radians_to_degrees(path->points[index].theta));

	}
	printf("NOVO\n");
	for (index = 0; index < length; index++)
	{
		printf("index: %d\tphi: %.2f\t x: %.2f y %.2f theta: %.2f\n",index, carmen_radians_to_degrees(best_points[index].phi), best_points[index].x,best_points[index].y, carmen_radians_to_degrees(best_points[index].theta));

	}
	usleep(50000000);




}

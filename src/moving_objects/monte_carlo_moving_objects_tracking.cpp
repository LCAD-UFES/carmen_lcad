#include <math.h>
#include <vector>
#include <list>

#include "moving_objects.h"
#include "monte_carlo_moving_objects_tracking.h"

using std::vector;
using namespace std;

double alpha_1 = 1.0; // desvio padrão da velocidade padrão 0.2
double alpha_2 = 0.1; // desvio padrão de theta padrão 0.01
double v_min = 0.0; // velocidade mínima;
double v_max = 25.0; // velocidade máxima


////////////////////////////////////////////////////////////////////////////////////////////


particle
sample_motion_model(double delta_time, particle particle_t_1)
{

	particle particle_t;
	carmen_point_t pose_t;
	carmen_point_t pose_t_1;
	double v;

	pose_t_1.x = particle_t_1.pose.x;
	pose_t_1.y = particle_t_1.pose.y;
	v = particle_t_1.velocity;
	pose_t_1.theta = particle_t_1.pose.theta;

	v = v + carmen_gaussian_random(0.0, alpha_1);
	if (v > v_max)
		v = v_max;
	if (v < v_min)
		v = v_min;

	pose_t_1.theta = pose_t_1.theta + carmen_gaussian_random(0.0, alpha_2);
	pose_t_1.theta = carmen_normalize_theta(pose_t_1.theta);

	pose_t.x = pose_t_1.x + delta_time * v * cos(pose_t_1.theta);
	pose_t.y = pose_t_1.y + delta_time * v * sin(pose_t_1.theta);

	particle_t.pose.theta = carmen_normalize_theta(pose_t_1.theta);
	particle_t.pose.x = pose_t.x;
	particle_t.pose.y = pose_t.y;
	particle_t.velocity = v;
	particle_t.weight = particle_t_1.weight;

	return particle_t;
}


double
distance_to_the_nearest_neighbor(double x_z_t, double y_z_t, carmen_point_t pose_t)
{
	double distance;
	double d_x, d_y;

	d_x = x_z_t - pose_t.x;
	d_y = y_z_t - pose_t.y;

	distance = sqrt ((d_x) * (d_x)) + ((d_y) * (d_y));

	return(distance);
}


double
calculation_particle_weight_pose_reading_model(double dist)
{
	double particle_weight;

	particle_weight = exp (-dist);

	return particle_weight;
}



void
measurement_model(double x, double y, std::vector<particle> *temporary_particle_set_t)
{

	int num_particles = temporary_particle_set_t->size();
	double *distance = (double*)malloc(sizeof(double) * num_particles);
	double sum1 = 0.0;

	int i = 0;
	for(std::vector<particle>::iterator it = temporary_particle_set_t->begin();
			it != temporary_particle_set_t->end(); it++)
	{
		distance[i] = distance_to_the_nearest_neighbor(x, y, it->pose);
		i++;
	}

	i = 0;
	for(std::vector<particle>::iterator it = temporary_particle_set_t->begin();
			it != temporary_particle_set_t->end(); it++)
	{
		it->weight = calculation_particle_weight_pose_reading_model(distance[i]);
		sum1 += it->weight;
		i++;
	}

	//normalização do peso
	for(std::vector<particle>::iterator it = temporary_particle_set_t->begin();
			it != temporary_particle_set_t->end(); it++)
	{
		it->weight = (it->weight / sum1);
	}

	free(distance);
}


/* resample particle filter */
void resample(std::vector<particle> *particle_set_t)
{
	static double *cumulative_sum = NULL;
	static particle *temp_particles = NULL;
	static int num_particles = particle_set_t->size();

	particle *aux;
	particle *copy_particle_set_t = NULL;
	int i, which_particle;
	double weight_sum;
	double position, step_size;
//	double max_weight;

	/* Allocate memory necessary for resampling */
	cumulative_sum = (double *)calloc(num_particles, sizeof(double));
	carmen_test_alloc(cumulative_sum);
	temp_particles = (particle*)malloc(sizeof(particle) * num_particles);
	carmen_test_alloc(temp_particles);
	copy_particle_set_t = (particle*)malloc(sizeof(particle) * num_particles);
	carmen_test_alloc(copy_particle_set_t);

	int j = 0;
	for (std::vector<particle>::iterator it = particle_set_t->begin();
			it != particle_set_t->end(); it++)
	{
		copy_particle_set_t[j].pose.x = it->pose.x;
		copy_particle_set_t[j].pose.y = it->pose.y;
		copy_particle_set_t[j].pose.theta = it->pose.theta;
		copy_particle_set_t[j].velocity = it->velocity;
		copy_particle_set_t[j].weight = it->weight;
		j++;
	}

	weight_sum = 0.0;
//	max_weight = copy_particle_set_t[0].weight;
	/* change log weights back into probabilities */
//	for (i = 0; i < num_particles; i++)
//	{
//		if (copy_particle_set_t[i].weight > max_weight)
//			max_weight = copy_particle_set_t[i].weight;
//	}

	for(i = 0; i < num_particles; i++)
	{

		/* Sum the weights of all of the particles */
		weight_sum += copy_particle_set_t[i].weight;
		cumulative_sum[i] = weight_sum;
	}

	/* choose random starting position for low-variance walk */
	position = carmen_uniform_random(0, weight_sum);
	step_size = weight_sum / (double) num_particles;
	which_particle = 0;

	/* draw num_particles random samples */
	for(i = 0; i < num_particles; i++)
	{
		position += step_size;

		if (position > weight_sum)
		{
			position -= weight_sum;
			which_particle = 0;
		}

		while(position > cumulative_sum[which_particle])
			which_particle++;

		temp_particles[i] = copy_particle_set_t[which_particle];
	}

	/* Switch particle pointers */
	aux = copy_particle_set_t;
	copy_particle_set_t = temp_particles;
	temp_particles = aux;

	int m = 0;
	for (std::vector<particle>::iterator it = particle_set_t->begin();
			it != particle_set_t->end(); it++)
	{
		it->pose.x = copy_particle_set_t[m].pose.x;
		it->pose.y = copy_particle_set_t[m].pose.y;
		it->pose.theta = copy_particle_set_t[m].pose.theta;
		it->velocity = copy_particle_set_t[m].velocity;
		it->weight = copy_particle_set_t[m].weight;
		m++;
	}

	free(temp_particles);
	free(cumulative_sum);
	free(copy_particle_set_t);
}

double
calculate_degeneration_of_the_particles(std::vector<particle> *particle_set_t)
{
	double effective_sample_size;
	double sum_of_squared_weights = 0.0;

	for (std::vector<particle>::iterator it = particle_set_t->begin();
			it != particle_set_t->end(); it++)
	{
		sum_of_squared_weights += it->weight * it->weight;
	}

	effective_sample_size = 1 / sum_of_squared_weights;

	return effective_sample_size;
}

std::vector<particle>
algorithm_monte_carlo(std::vector<particle> *particle_set_t_1, double x, double y,
		double delta_time, pcl::PointCloud<pcl::PointXYZ> pcl_cloud)
{
	pcl::PointCloud<pcl::PointXYZ> aux_pcl_cloud = pcl_cloud;

	std::vector<particle> temporary_particle_set_t;
	std::vector<particle> particle_set_t;
//	double effective_sample_size;

	//prediction
	for(std::vector<particle>::iterator it = particle_set_t_1->begin();
			it != particle_set_t_1->end(); it++)
	{
		particle particle_t;
		particle particle_t_1;
		particle_t_1.pose.x = it->pose.x;
		particle_t_1.pose.y = it->pose.y;
		particle_t_1.pose.theta = it->pose.theta;
		particle_t_1.velocity = it->velocity;
		particle_t_1.weight = it->weight;

		particle_t = sample_motion_model(delta_time, particle_t_1);
		temporary_particle_set_t.push_back(particle_t);
	}

	//observation model
	measurement_model(x, y, &temporary_particle_set_t);

	//resampling
//	resample(&temporary_particle_set_t);

//	effective_sample_size = calculate_degeneration_of_the_particles(&temporary_particle_set_t);
//
//	if (effective_sample_size < 10.5)
//		resample(&temporary_particle_set_t);

	resample(&temporary_particle_set_t);
	particle_set_t = temporary_particle_set_t;

	return particle_set_t;
}

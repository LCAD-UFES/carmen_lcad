//#include "pso.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#define DATA_SIZE 50

#define NUM_ITERACTIONS 700
#define num_particles 70

#define dim 2

double velocities[num_particles][dim];
double particles[num_particles][dim];
double pbest[num_particles][dim];
double pbest_fitness[num_particles];
double fitness[num_particles];
double gbest[dim];
double gbest_fitness;

double data[DATA_SIZE];
double limits[dim][2];


double
fitness_function(double particle[])
{
	int i;
	double fitness;
	double pred;

	fitness = 0;

	for (i = 0; i < DATA_SIZE; i++)
	{
		pred = particle[0] * i + particle[1];
		fitness += fabs(pred - data[i]);
	}

	fitness = -fitness;
	return fitness;
}


void
read_data()
{
	int i;

	double a = 0.79;
	double b = 15.0;

	for (i = 0; i < DATA_SIZE; i++)
	{
		data[i] = a * i + b;
	}
}


void
set_limits()
{
	limits[0][0] = -3.1415;
	limits[0][1] = 3.1415;

	limits[1][0] = -100.0;
	limits[1][1] = 100.0;
}


void
randomize_particles()
{
	int i, j;

	for (i = 0; i < num_particles; i++)
	{
		for (j = 0; j < dim; j++)
		{
			particles[i][j] = ((double) rand() / (double) RAND_MAX) * (limits[j][1] - limits[j][0]) + (limits[j][0]);
		}
	}
}


void
initialize_pbest()
{
	int i, j;

	for (i = 0; i < num_particles; i++)
	{
		for (j = 0; j < dim; j++)
		{
			pbest[i][j] = particles[i][j];
		}

		pbest_fitness[i] = -DBL_MAX;
	}
}


void
intialize_gbest()
{
	gbest_fitness = -DBL_MAX;
}


void
initialize_velocities()
{
	int i, j;

	for (i = 0; i < num_particles; i++)
	{
		for (j = 0; j < dim; j++)
		{
			velocities[i][j] = 0.0;
		}
	}
}


void
initialize()
{
	set_limits();
	initialize_velocities();
	randomize_particles();
	initialize_pbest();
	intialize_gbest();
}


void
evaluate_fitness()
{
	int i;

	for (i = 0; i < num_particles; i++)
		fitness[i] = fitness_function(particles[i]);
}


void
copy(double from[], double to[])
{
	int i;

	for (i = 0; i < dim; i++)
	{
		to[i] = from[i];
	}
}


void
update_pbest()
{
	int i;

	for (i = 0; i < num_particles; i++)
	{
		if (fitness[i] > pbest_fitness[i])
		{
			pbest_fitness[i] = fitness[i];
			copy(particles[i], pbest[i]);
		}
	}
}


void
update_gbest()
{
	int i;

	for (i = 0; i < num_particles; i++)
	{
		if (fitness[i] > gbest_fitness)
		{
			gbest_fitness = fitness[i];
			copy(particles[i], gbest);			
		}
	}
}


void
update_particles()
{
	int i, j;
	double r1, r2;
	double social_component;
	double cognitive_component;
	double historical_component;
	
	double c2 = 2.05;
	double c1 = 2.05;
	double phi = c1 + c2;
	double constriction = 2 / fabs((2 - phi - sqrt(pow(phi, 2) - 4 * phi)));

	for (i = 0; i < num_particles; i++)
	{
		for (j = 0; j < dim; j++)
		{
			r1 = (double) rand() / (double) RAND_MAX;
			r2 = (double) rand() / (double) RAND_MAX;
			
			historical_component = velocities[i][j];
			social_component = c2 * r2 * (gbest[j] - particles[i][j]);
			cognitive_component = c1 * r1 * (pbest[i][j] - particles[i][j]);

			// TODO: check for max and min velocities
			velocities[i][j] = constriction * (historical_component + cognitive_component + social_component);
			particles[i][j] = particles[i][j] + velocities[i][j];

			// check for particle limits
			if (particles[i][j] > limits[j][1])
				particles[i][j] = limits[j][1];
			if (particles[i][j] < limits[j][0])
				particles[i][j] = limits[j][0];
		}
	}
}


int 
main(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
	int n = 0;

	read_data();
	initialize();

	while (n < NUM_ITERACTIONS)
	{
		evaluate_fitness();
		update_pbest();
		update_gbest();
		update_particles();

		n++;
	}

	printf("gbest: %lf %lf fitness: %lf\n", gbest[0], gbest[1], gbest_fitness);
	return 0;
}


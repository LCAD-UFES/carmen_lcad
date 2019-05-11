#include "pso.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


double 
fitness(double *particle, void *data __attribute__ ((unused)), int particle_id __attribute__ ((unused)))
{
	int i;
	double x;
	double fitness = 1;

	for (i = 0; i < 10; i++)
	{
		x = (double) particle[i];
		fitness *= (x * sin(x));
	}

	return fitness;
}


double**
set_limits()
{
	int i;
	double **limits;

	limits = (double **) calloc (10, sizeof(double*));
	
	for (i = 0; i < 10; i++)
	{
		limits[i] = (double *) calloc (2, sizeof(double));

		limits[i][0] = -100;
		limits[i][1] = 100;
	}

	return limits;
}


int 
main(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
	double **limits = set_limits();

	ParticleSwarmOptimization optimizer(
		fitness, 
		limits, 
		10, NULL, 
		410, 2000);

	optimizer.Optimize();
	
	printf("Result: %lf Fitness: %lf\n", optimizer.GetBestSolution()[0], optimizer.GetBestFitness());

	double *error = optimizer.GetErrorEvolution();
	int i;
	FILE *f = fopen("error2.txt", "w");
	for (i = 0; i < 2000; i++)
	{
		fprintf(f, "%lf\n", error[i]);
	}
	fclose(f);

	return 0;
}


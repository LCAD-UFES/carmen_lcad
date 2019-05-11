#include <omp.h>
#include "pso.h"
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <time.h>
#include <carmen/carmen.h>


ParticleSwarmOptimization::ParticleSwarmOptimization(double (*fitness_function)(double *particle, void *data, int particle_id), double **limits, int dim, void *data, int num_particles, int max_iteractions)
{
	srand(time(NULL));

	_fitness_function = fitness_function;
	_max_iteractions = max_iteractions;
	_num_particles = num_particles;
	_limits = limits;
	_data = data;
	_dim = dim;

	_alloc_stuff();
	_initialize_velocities();
	_randomize_particles();
	_initialize_pbest();
	_intialize_gbest();
}


ParticleSwarmOptimization::~ParticleSwarmOptimization()
{
	_dealloc_stuff();
}


void
ParticleSwarmOptimization::Optimize(void (*interaction_function)(ParticleSwarmOptimization *opt, void *data, int particle_id))
{
	int num_iteractions = 0;

	while (num_iteractions < _max_iteractions)
	{
		_evaluate_fitness();
		_update_pbest();
		_update_gbest();
		_update_particles();
//		int random_particle = _put_a_random_particle_near_gbest();
//		if ((num_iteractions % 200) == 0)
//		{
//			printf("***************************************************************************\n");
//			_initialize_velocities();
//			_randomize_particles();
//			_initialize_pbest();
//			_copy(_particles[random_particle], _gbest);
//		}
		_error[num_iteractions] = _gbest_fitness;

		fprintf(stderr, "Iteraction: %d of %d GbestFitness: %lf\n", 
			num_iteractions, _max_iteractions, -_gbest_fitness);

		if (interaction_function)
			(*interaction_function)(this, _data, _best_particle_id);

		num_iteractions++;
	}
}


double 
ParticleSwarmOptimization::GetBestFitness()
{
	return _gbest_fitness;
}


double*
ParticleSwarmOptimization::GetBestSolution()
{
	return _gbest;
}


double*
ParticleSwarmOptimization::GetErrorEvolution()
{
	return _error;
}


int
ParticleSwarmOptimization::GetBestParticleId()
{
	return _best_particle_id;
}


void 
ParticleSwarmOptimization::_alloc_stuff()
{
	int i;

	_velocities = (double **) calloc (_num_particles, sizeof(double*)); 
	_particles = (double **) calloc (_num_particles, sizeof(double*)); 
	_pbest = (double **) calloc (_num_particles, sizeof(double*)); 

	_pbest_fitness = (double *) calloc (_num_particles, sizeof(double)); 
	_fitness = (double *) calloc (_num_particles, sizeof(double)); 
	_gbest = (double *) calloc (_dim, sizeof(double)); 
	_error = (double *) calloc (_max_iteractions, sizeof(double)); 

	for (i = 0; i < _num_particles; i++)
	{
		_velocities[i] = (double *) calloc (_dim, sizeof(double)); 
		_particles[i] = (double *) calloc (_dim, sizeof(double)); 
		_pbest[i] = (double *) calloc (_dim, sizeof(double)); 
	}
}


void 
ParticleSwarmOptimization::_dealloc_stuff()
{
	int i;

	for (i = 0; i < _num_particles; i++)
	{
		free(_velocities[i]);
		free(_particles[i]);
		free(_pbest[i]);
	}

	free(_velocities);
	free(_particles);
	free(_pbest);

	free(_pbest_fitness);
	free(_fitness);
	free(_gbest);
	free(_error);
}


void
ParticleSwarmOptimization::_initialize_velocities()
{
	int i, j;

	for (i = 0; i < _num_particles; i++)
		for (j = 0; j < _dim; j++)
			_velocities[i][j] = 0.0;
}


void
ParticleSwarmOptimization::_randomize_particles()
{
	int i, j;

	for (i = 0; i < _num_particles; i++)
		for (j = 0; j < _dim; j++)
			_particles[i][j] = ((double) rand() / (double) RAND_MAX) * (_limits[j][1] - _limits[j][0]) + _limits[j][0];
}


void
ParticleSwarmOptimization::_initialize_pbest()
{
	int i, j;

	for (i = 0; i < _num_particles; i++)
	{
		for (j = 0; j < _dim; j++)
			_pbest[i][j] = _particles[i][j];

		_pbest_fitness[i] = -DBL_MAX;
	}
}


void
ParticleSwarmOptimization::_intialize_gbest()
{
	_gbest_fitness = -DBL_MAX;
	_best_particle_id = -1;
}


void
ParticleSwarmOptimization::_evaluate_fitness()
{
	int i;

	// it's highly parallelizable!!
	// ATENCAO! Para usar OpenMP a sua fitness fucntion precisa ser thread safe!
	#pragma omp parallel for default(shared) private(i)
	for (i = 0; i < _num_particles; i++)
		_fitness[i] = _fitness_function(_particles[i], _data, i);
}


void
ParticleSwarmOptimization::_update_pbest()
{
	int i;

	for (i = 0; i < _num_particles; i++)
	{
		if (_fitness[i] > _pbest_fitness[i])
		{
			_pbest_fitness[i] = _fitness[i];
			_copy(_particles[i], _pbest[i]);
		}
	}
}


void
ParticleSwarmOptimization::_update_gbest()
{
	int i;

	for (i = 0; i < _num_particles; i++)
	{
		if (_fitness[i] > _gbest_fitness)
		{
			_gbest_fitness = _fitness[i];
			_copy(_particles[i], _gbest);			
			_best_particle_id = i;
		}
	}
}


double
gaussian_random_pso(double mean, double std)
{
  const double norm = 1.0 / (RAND_MAX + 1.0);
  double u = 1.0 - rand() * norm;                  /* can't let u == 0 */
  double v = rand() * norm;
  double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
  return (mean + std * z);
}


int
ParticleSwarmOptimization::_put_a_random_particle_near_gbest()
{
//	static bool first_time = true;
//	static double *previous_gbest;
//
//	if (first_time)
//	{
//		previous_gbest = (double *) calloc (_dim, sizeof(double));
//		_copy(_gbest, previous_gbest);
//
//		first_time = false;
//		return;
//	}

	int random_particle = rand() % _num_particles;
	_copy(_gbest, _particles[random_particle]);
	int j = rand() % _dim;
//	for (int j = 0; j < _dim; j++)
	{
		_particles[random_particle][j] = _gbest[j] + (_gbest[j] - _particles[random_particle][j]) * 0.01 * (2.0 * ((double) rand() / (double) RAND_MAX) - 1.0);
		_particles[random_particle][j] += (_limits[j][1] - _limits[j][0]) * 0.01 * (2.0 * ((double) rand() / (double) RAND_MAX) - 1.0);
		// check for particle limits
		if (_particles[random_particle][j] > _limits[j][1])
			_particles[random_particle][j] = _limits[j][1];
		if (_particles[random_particle][j] < _limits[j][0])
			_particles[random_particle][j] = _limits[j][0];

//		_velocities[random_particle][j] = 0.0;
	}
	for (int j = 0; j < _dim; j++)
	{
		printf("%lf %lf\n", _particles[random_particle][j], _gbest[j]);
		_velocities[random_particle][j] = 0.0;
	}
	printf("\n");

	return (random_particle);
}


void
ParticleSwarmOptimization::_update_particles()
{
	int i, j;
	double r1, r2;
	double social_component;
	double cognitive_component;
	double historical_component;
	
	double c2 = 2.05;
	double c1 = 2.05;
	double phi = c1 + c2;
	double constriction = 2.0 / fabs((2.0 - phi - sqrt(pow(phi, 2.0) - 4.0 * phi)));

	for (i = 0; i < _num_particles; i++)
	{
		for (j = 0; j < _dim; j++)
		{
			r1 = (double) rand() / (double) RAND_MAX;
			r2 = (double) rand() / (double) RAND_MAX;
			
			historical_component = _velocities[i][j];
			social_component = c2 * r2 * (_gbest[j] - _particles[i][j]);
			cognitive_component = c1 * r1 * (_pbest[i][j] - _particles[i][j]);

			double mean =  (_gbest[j] + _pbest[i][j]) / 2.0;
			double std = fabs(_gbest[j] - _pbest[i][j]);
			_particles[i][j] = gaussian_random_pso(mean, std);

			// TODO: check for max and min _velocities
			_velocities[i][j] = constriction * (historical_component + cognitive_component + social_component);
//			_particles[i][j] = _particles[i][j] + _velocities[i][j];

			// check for particle limits
			if (_particles[i][j] > _limits[j][1])
				_particles[i][j] = _limits[j][1];
			if (_particles[i][j] < _limits[j][0])
				_particles[i][j] = _limits[j][0];
		}
	}
}


void
ParticleSwarmOptimization::_copy(double from[], double to[])
{
	int i;

	for (i = 0; i < _dim; i++)
		to[i] = from[i];
}

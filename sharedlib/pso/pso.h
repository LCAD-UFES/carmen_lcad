#ifndef _PSO_H_
#define _PSO_H_

#include <stdlib.h>

class ParticleSwarmOptimization
{
public:

	ParticleSwarmOptimization(double (*fitness_function)(double *particle, void *data, int particle_id), double **limits, int dim, void *data = NULL, int num_particles = 70, int max_iteractions = 100);
	~ParticleSwarmOptimization();
	
	void Optimize(void (*interaction_function)(ParticleSwarmOptimization *opt, void *data, int particle_id) = NULL);
	double GetBestFitness();
	double *GetBestSolution();
	double *GetErrorEvolution();
	int GetBestParticleId();

protected:

	void _alloc_stuff();
	void _dealloc_stuff();
	void _initialize_velocities();
	void _randomize_particles();
	void _initialize_pbest();
	void _intialize_gbest();
	void _evaluate_fitness();
	void _update_pbest();
	void _update_gbest();
	void _update_particles();
	int _put_a_random_particle_near_gbest();
	void _copy(double from[], double to[]);
	
	int _max_iteractions; 
	int _num_particles;
	int _dim;

	double **_velocities;	// [num_particles][dim];
	double **_particles;	// [num_particles][dim];
	double **_pbest;		// [num_particles][dim];
	double **_limits;		// [dim][2];

	double *_pbest_fitness;	// [num_particles];
	double *_fitness;		// [num_particles];
	double *_gbest;			// [dim];
	double *_error;			// [num_iteractions]

	double _gbest_fitness;
	int _best_particle_id;

	double (*_fitness_function)(double *particle, void *data, int particle_id);
	void *_data;
};

#endif 

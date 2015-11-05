#ifndef CARMEN_MONTE_CARLO_LOCALIZATION_MAIN_H
#define CARMEN_MONTE_CARLO_LOCALIZATION_MAIN_H

#include <carmen/carmen.h>
#include <carmen/simulator_ackerman_interface.h>
#include <carmen/velodyne_interface.h>
#include "monte_carlo_localization_interface.h"

typedef struct particle{

	carmen_point_t pose;
	double weigth;
} particle;

typedef struct
_likelihood_field_range_finder_measurement_model_params
{
	double zhit, zshort, zmax, zrand;
	double sigma_zhit;
	double lambda_short;
	double max_range;
	double fov_range;
	double angle_step;
	double start_angle;
	int sampling_step;
	int laser_beams;
	double front_offset;
	double side_offset;
	double angular_offset;
	double l0, lfree, locc;
	double lambda_short_min, lambda_short_max;
} likelihood_field_range_finder_measurement_model_params;

#endif

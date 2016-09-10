#include <carmen/carmen.h>
#include <fann.h>
#include <fann_train.h>
#include <fann_data.h>
#include <floatfann.h>
#include "car_neural_model.h"


void
carmen_libcarneuralmodel_init_steering_ann_input(fann_type *input)
{
	int i;

	// steering = 0, current_curvature = 0
	for (i = 0; i < (NUM_STEERING_ANN_INPUTS - 1); i += 2)
	{
		input[i] = 0.0;
		input[i + 1] = 0.0;
	}
}


void
carmen_libcarneuralmodel_build_steering_ann_input(fann_type *input, double s, double cc)
{
	int i;

	for (i = 0; i < (NUM_STEERING_ANN_INPUTS - 2); i++)
		input[i] = input[i + 2];

	input[NUM_STEERING_ANN_INPUTS - 2] = s / 100.0;
	input[NUM_STEERING_ANN_INPUTS - 1] = cc;
}


/* NAO ESTA FUNCIONANDO PQ TEM Q RETORNAR O fann_create_from_file
void
carmen_libcarneuralmodel_init_steering_ann (struct fann *steering_ann, fann_type *steering_ann_input)
{
	if (steering_ann == NULL)
	{
		steering_ann = fann_create_from_file("steering_ann.net");
		if (steering_ann == NULL)
		{
			printf("Error: Could not create steering_ann\n");
			exit(1);
		}
		carmen_libcarneuralmodel_init_steering_ann_input(steering_ann_input);
	}
}
*/

/*
double
carmen_libcarneuralmodel_compute_new_phi (double phi, double target_phi, double v, double understeer_coeficient,
											double distance_between_front_and_rear_axles, double delta_t)
{
	double atan_current_curvature;
	double atan_desired_curvature;



}*/














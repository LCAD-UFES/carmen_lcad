#include "valj.h"
#include <fann.h>
#include <fann_train.h>
#include <fann_data.h>

/*
 * Função da camada oculta:  SIGMOID_STEPWISE
 * Função da camada de saída:  SIGMOID_SYMMETRIC
 * Quantidade de Neurônios Ocultos:  10
 * Algoritmo de Aprendizado:  TRAIN_RPROP
 * Learning Rate:  0.3
 * Momentum:  0.4
 * Quantidade de épocas a treinar:  200
 */

int 
main(int argc, char **argv)
{
	const unsigned int num_input = 4;
	const unsigned int num_output = 1;
	const unsigned int num_layers = 4;
	const unsigned int num_neurons_hidden = 12;
	const float desired_error = (const float) 0.000001;
	const unsigned int max_epochs = 200;
	const unsigned int epochs_between_reports = 10;
	struct fann *ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_neurons_hidden/2, num_output);
	unsigned int i, j;

	/*if (argc != 2)
	{ 
		fprintf(stderr, "Use: %s arquivoTreino\n", argv[0]); 
		exit(1);
	}*/

	fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
	fann_set_activation_function_output(ann, FANN_SIGMOID);
	fann_set_training_algorithm(ann, FANN_TRAIN_RPROP);
	fann_set_learning_rate(ann, 0.1);
	fann_set_learning_momentum(ann, 0.6);
    fann_randomize_weights(ann, -1.0, 1.0);

	fann_save(ann, ARQ_RNA);
	fann_destroy(ann);

	return 0;
}

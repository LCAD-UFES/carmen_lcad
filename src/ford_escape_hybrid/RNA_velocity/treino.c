#include "valj.h"
#include <fann.h>

int 
main(int argc, char **argv)
{
	const unsigned int num_input = 360;
	const unsigned int num_output = 1;
	const unsigned int num_layers = 4;
	const unsigned int num_neurons_hidden = num_input / 2;
	const float desired_error = (const float) 0.001;
	const unsigned int max_epochs = 300;
	const unsigned int epochs_between_reports = 10;
	struct fann_train_data * data = NULL;
	struct fann *ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_neurons_hidden / 2, num_output);
	unsigned int i, j;

	if (argc != 2)
	{
		fprintf(stderr, "Use: %s arquivoTreino\n", argv[0]); 
		exit(1);
	}

	fann_set_activation_function_hidden(ann, FANN_ELLIOT);
	fann_set_activation_function_output(ann, FANN_LINEAR);
	fann_set_training_algorithm(ann, FANN_TRAIN_RPROP);
	fann_set_learning_rate(ann, 0.1);
	fann_set_learning_momentum(ann, 0.6);
	data = fann_read_train_from_file(argv[1]);

	fann_train_on_data(ann, data, max_epochs, epochs_between_reports, desired_error);

	free(data);
	fann_save(ann, ARQ_RNA);
	fann_destroy(ann);

	return 0;
}

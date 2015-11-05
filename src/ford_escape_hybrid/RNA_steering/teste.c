#include "valj.h"
#include <stdio.h>
#include <unistd.h>
#include <fann.h>

int 
main(int argc, char **argv)
{
	fann_type *calc_out;
	unsigned int i, j;
	struct fann *ann;
	struct fann_train_data *data;

	if (argc < 2) 
	{
		fprintf(stderr, "Use: %s arquivoTeste\n", argv[0]); 
		exit(1);
	}

	printf("Abrindo a Rede `%s'\n", ARQ_RNA);
	ann = fann_create_from_file(ARQ_RNA);

	if (!ann)
	{
		fprintf(stderr, "Erro criando a RNA.\n"); 
		return (1); 
	}

	//fann_print_connections(ann);
	//fann_print_parameters(ann);

	printf("Testando a RNA.\n");

	data = fann_read_train_from_file(argv[1]);

	for(i = 0; i < fann_length_train_data(data); i++)
	{
		fann_reset_MSE(ann);

		calc_out = fann_run(ann, data->input[i]);

		printf("Resultado: %f ", calc_out[0]);
		printf("Original: %f " , data->output[i][0]);
		printf("Erro: %f\n"    , (float) fann_abs(calc_out[0] - data->output[i][0]));
	}
	printf("Limpando memoria.\n");
	fann_destroy_train(data);
	fann_destroy(ann);

	return (0);
}

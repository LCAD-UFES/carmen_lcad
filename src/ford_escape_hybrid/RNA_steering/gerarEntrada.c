#include <stdio.h>
#include <stdlib.h>


int 
main(int argc, char **argv)
{
	FILE *input_file;
	char line[1024];
	double cc, dc, e, integral, derivada, s;
	int num_time_lags;
	int num_lines;
	int i, j;
	int num_train_samples, num_inputs;
	double *input;

	if (argc != 3)
	{
		fprintf(stderr, "Use: %s arquivoPID janelaTempo\n", argv[0]);
		exit(1);
	}

	num_time_lags = atoi(argv[2]);
	input_file = fopen(argv[1], "r");

	// Count samples
	num_lines = 0;
	while (fgets(line, 1023, input_file) != NULL)
		num_lines++;
	rewind(input_file);

	// number of training pairs, number of inputs, number of outputs
	//
	// number of training pairs: num_lines - num_time_lags
	// number of inputs: (steering effort, atan_current_curvature) * num_time_lags = 2 item * num_time_lags
	// number of outputs: 1 (predicted atan_current_curvature)
	num_train_samples = num_lines - num_time_lags;
	num_inputs = num_time_lags * 2;
	printf("%d %d %d\n", num_train_samples, num_inputs, 1);

	input = (double *) malloc(num_inputs * sizeof(double));
	i = 0;
	while (fgets(line, 1023, input_file) != NULL)
	{
		// STEERING (cc, dc, e, i, d, s): -0.000338, -0.000000, 0.000338, 0.000043, 0.000000, 1.878823
		sscanf(line, "STEERING (cc, dc, e, i, d, s): %lf, %lf, %lf, %lf, %lf, %lf", &cc, &dc, &e, &integral, &derivada, &s);

		if (i >= num_inputs)
		{
			for (j = 0; j < num_inputs; j++)
				printf("%lf ", input[j]);
			printf("\n%lf\n", cc);

			for (j = 0; j < (num_inputs - 2); j++)
			{
				input[j] = input[j + 2];
			}
			input[num_inputs - 2] = s / 100.0;
			input[num_inputs - 1] = cc;
		}
		else
		{
			input[i++] = s / 100.0;
			input[i++] = cc;
		}
	}
	return (0);
}

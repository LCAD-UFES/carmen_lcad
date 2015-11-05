#include <stdio.h>
#include <stdlib.h>


int 
main(int argc, char **argv)
{
	FILE *input_file;
	char line[1024];
	double cv, dv, e, t, b, cv_from_file;
//	double integral, derivada;
	int st;
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
	// number of inputs: (trottle, brake, cv) * num_time_lags = 3 itens * num_time_lags
	// number of outputs: 1 (predicted v)
	num_train_samples = num_lines - num_time_lags;
	num_inputs = num_time_lags * 3;
	printf("%d %d %d\n", num_train_samples, num_inputs, 1);

	input = (double *) malloc(num_inputs * sizeof(double));
	i = 0;
	while (fgets(line, 1023, input_file) != NULL)
	{
		// VELOCITY (st, cv, dv, e, t, b): 0, 0.008662, 0.000000, -0.008662, 0.000000, 100.000000
		// VELOCITY (st, cv, dv, e, i, d, t, b): 0, 0.058467, 0.000000, -0.058467, 0.000000, -1.465864, 0.000000, 50.000000
		sscanf(line, "VELOCITY (st, cv, dv, e, t, b): %d, %lf, %lf, %lf, %lf, %lf", &st, &cv, &dv, &e, &t, &b);
		// sscanf(line, "VELOCITY (st, cv, dv, e, i, d, t, b): %d, %lf, %lf, %lf, %lf, %lf, %lf, %lf", &st, &cv, &dv, &e, &integral, &derivada, &t, &b);

		if (i >= num_inputs)
		{
			for (j = 0; j < num_inputs; j++)
				printf("%lf ", input[j]);
			printf("\n%lf\n", cv);

			for (j = 0; j < (num_inputs - 3); j++)
			{
				input[j] = input[j + 3];
			}
			input[num_inputs - 3] = t / 100.0;
			input[num_inputs - 2] = b / 100.0;
			input[num_inputs - 1] = cv / 5.0;
		}
		else
		{
			input[i++] = t / 100.0;
			input[i++] = b / 100.0;
			input[i++] = cv / 5.0;
		}
	}
	return (0);
}

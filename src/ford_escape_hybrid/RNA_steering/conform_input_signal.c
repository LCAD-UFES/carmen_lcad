#include <stdio.h>
#include <stdlib.h>
#include <math.h>


int 
main(int argc, char **argv)
{
	FILE *input_file;
	char line[1024];
	double cc, dc, e, integral, derivada, s;

	if (argc != 2)
	{
		fprintf(stderr, "Use: %s arquivoPID\n", argv[0]);
		exit(1);
	}

	input_file = fopen(argv[1], "r");

	while (fgets(line, 1023, input_file) != NULL)
	{
		// STEERING (cc, dc, e, i, d, s): -0.000338, -0.000000, 0.000338, 0.000043, 0.000000, 1.878823
		sscanf(line, "STEERING (cc, dc, e, i, d, s): %lf, %lf, %lf, %lf, %lf, %lf", &cc, &dc, &e, &integral, &derivada, &s);
		if (fabs(cc) < 0.001)
			printf("STEERING (cc, dc, e, i, d, s): %lf, %lf, %lf, %lf, %lf, %lf\n", 0.0, dc, e, integral, derivada, 0.0);
		else
			printf("STEERING (cc, dc, e, i, d, s): %lf, %lf, %lf, %lf, %lf, %lf\n", cc, dc, e, integral, derivada, s);

	}
	return (0);
}

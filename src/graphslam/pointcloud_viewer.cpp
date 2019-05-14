
#include <stdio.h>
#include <stdlib.h>

FILE *gnuplot_pipe;


void
plot_graph(char *file_name)
{
	static bool first_time = true;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipe = popen("gnuplot", "w"); // -persist to keep last plot after program closes
//		fprintf(gnuplot_pipe, "set size square\n");
//		fprintf(gnuplot_pipe, "set size ratio -1\n");
		fprintf(gnuplot_pipe, "set xrange [-50:50]\n");
		fprintf(gnuplot_pipe, "set yrange [-50:50]\n");
		fprintf(gnuplot_pipe, "set zrange [-3:20]\n");
//		fprintf(gnuplot_pipe, "set xlabel 'senconds'\n");
//		fprintf(gnuplot_pipe, "set ylabel 'effort'\n");
	}

	fprintf(gnuplot_pipe, "splot '%s' u 1:2:3\n", file_name);
	fflush(gnuplot_pipe);
}


int 
main(int argc, char **argv)
{
	char filename[1024];

	if (argc < 2)
		exit(printf("Error: use %s <file with list of pointclouds>\n", argv[0]));

	FILE *f = fopen(argv[1], "r");

	while (!feof(f))
	{
		//fgets(filename, 1023, f);
		fscanf(f, "\n%s\n", filename);
		plot_graph(filename);
		getchar();
	}

	fclose(gnuplot_pipe);
	return (0);
}

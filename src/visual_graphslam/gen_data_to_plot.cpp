#include <stdio.h>
#include "g2o/types/slam2d/se2.h"
using namespace g2o;

double optx, opty, opttheta, opt_trash;
double syncx, syncy, synctheta;
double sync_trash01, sync_trash02, sync_trash03;
double sync_trash04, sync_trash05, sync_trash06, sync_trash07;

int is_first_line = 1;
double firstx, firsty;

SE2 error_point1;
SE2 error_point2;


void
read_sync_line(FILE *sync_file)
{
	fscanf(sync_file, "\n%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
		&sync_trash01, &sync_trash02, &sync_trash03,
		&syncx, &syncy, &synctheta,
		&sync_trash04, &sync_trash05, &sync_trash06, &sync_trash07
	);
}


void
read_opt_line(FILE *opt_file)
{
	fscanf(opt_file, "\n%lf %lf %lf %lf\n", &optx, &opty, &opttheta, &opt_trash);
}


void
process_first_line()
{
	firstx = syncx;
	firsty = syncy;
	is_first_line = 0;
}


void
pre_process_data()
{
	syncx -= firstx;
	syncy -= firsty;

	optx -= firstx;
	opty -= firsty;
}


void
compute_error_points()
{
	// transform the points 0,5 and 0,-5 to the car pose
	error_point1 = SE2(syncx, syncy, synctheta) * SE2(0.0, 5.0, 0.0);
	error_point2 = SE2(syncx, syncy, synctheta) * SE2(0.0, -5.0, 0.0);
}


void
print_result()
{
	printf("%lf %lf %lf %lf %lf %lf %lf %lf\n",
		syncx, syncy, optx, opty,
		error_point1[0], error_point1[1],
		error_point2[0], error_point2[1]
	);
}


void
process_lines()
{
	pre_process_data();
	compute_error_points();
	print_result();
}


void
read_and_process_line(FILE *sync_file, FILE *opt_file)
{
	read_sync_line(sync_file);
	read_opt_line(opt_file);

	if (is_first_line)
		process_first_line();

	process_lines();
}


int
main(int argc, char **argv)
{
	FILE *sync_file, *opt_file;

	if (argc < 3)
		exit(printf("Use %s <sync.txt> <opt.txt>\n", argv[0]));

	sync_file = fopen(argv[1], "r");
	opt_file = fopen(argv[2], "r");

	if (sync_file == NULL) exit(printf("sync_file = '%s' returned NULL!\n", argv[1]));
	if (opt_file == NULL) exit(printf("opt_file = '%s' returned NULL!\n", argv[2]));

	while ((!feof(sync_file)) && (!feof(opt_file)))
		read_and_process_line(sync_file, opt_file);

	fclose(sync_file);
	fclose(opt_file);

	return 0;
}


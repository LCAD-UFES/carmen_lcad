#include <locale.h>
#include <carmen/carmen.h>


static char *carmen_rddf_filename;
static double carmen_rddf_min_distance_between_waypoints = 0.5;
FILE *fptr = NULL;


static void
localize_globalpos_from_file(FILE *fptr_input)
{
	double v, phi, timestamp, distance;
	carmen_point_t current_pose;
	static carmen_point_t last_pose;
	static int first_time = 1;
	FILE *fptr = NULL;

	while(!feof(fptr_input))
	{
		fscanf(fptr_input, "%lf %lf %lf %lf %lf %lf\n",
			&current_pose.x, &current_pose.y, &current_pose.theta, 
			&v, &phi, &timestamp);
		if (first_time)
		{
			distance = carmen_rddf_min_distance_between_waypoints;
			first_time = 0;
		}
	else
		distance = DIST2D(current_pose, last_pose);

	if (distance < carmen_rddf_min_distance_between_waypoints)
		return;

	last_pose = current_pose;

	fptr = fopen(carmen_rddf_filename, "a");
	fprintf(fptr, "%lf %lf %lf %lf %lf %lf\n",
			current_pose.x, current_pose.y, current_pose.theta, 
			v, phi, timestamp);
	fclose(fptr);
	}
}


int
main (int argc, char **argv)
{
	printf("This script standardizes localizer poses according to the minimum distance\n");

	if (argc < 2)
		exit(printf("Use %s <rddf-file> <poses-input: x y theta v phi timestamp> <min-distance-between-waypoints>\n", argv[0]));

	FILE *fptr_input = NULL;
	carmen_rddf_filename = argv[1];
	carmen_rddf_min_distance_between_waypoints = atof(argv[3]);

	setlocale(LC_ALL, "C");

	// just to clean the file
	fptr = fopen(carmen_rddf_filename, "w");
	fclose(fptr);

	fptr_input = fopen(argv[2], "r");
	localize_globalpos_from_file(fptr_input);
	fclose(fptr_input);

	return (0);
}

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <carmen/carmen.h>
#include <math.h>


struct rddf_line_t
{
	double x, y, yaw, v, phi, timestamp;
};

using namespace std;


struct rddf_line_t
get_new_sample(struct rddf_line_t rddf_sample, double d_interval, double d_time, double yaw)
{
	double coss, sine;

	sincos(yaw, &sine, &coss);
	rddf_sample.x = rddf_sample.x + d_interval * coss;
	rddf_sample.y = rddf_sample.y + d_interval * sine;
	rddf_sample.timestamp = rddf_sample.timestamp + d_time;

	return (rddf_sample);
}


int
main(int argc, char **argv)
{
	if (argc != 2)
	{
		printf("Usage:\n rddf_reasample <rddf.txt>\n");
		exit(1);
	}

	FILE *fptr = fopen(argv[1], "r");
	if (!fptr)
	{
		printf("Could not open rddf file: %s\n", argv[1]);
		exit(1);
	}

	struct rddf_line_t rddf_line;
	vector<struct rddf_line_t> rddf;
	while (!feof(fptr))
	{
		fscanf(fptr, "%lf %lf %lf %lf %lf %lf\n",
			&(rddf_line.x), &(rddf_line.y),	&(rddf_line.yaw), &(rddf_line.v), &(rddf_line.phi),	&(rddf_line.timestamp));
		rddf.push_back(rddf_line);
	}

	for (unsigned int i = 0; i < rddf.size() - 1; i++)
	{
		double distance = DIST2D(rddf[i], rddf[i + 1]);
		int intervals = distance / 0.5;
		double d_interval = distance / (double) intervals;
		double d_time = (rddf[i + 1].timestamp - rddf[i].timestamp) / (double) intervals;

		struct rddf_line_t rddf_sample = rddf[i];
		for (int j = 0; j < intervals; j++)
		{
			printf("%lf %lf %lf %lf %lf %lf\n",
				(rddf_sample.x), (rddf_sample.y), (rddf_sample.yaw), (rddf_sample.v), (rddf_sample.phi), (rddf_sample.timestamp));
			rddf_sample = get_new_sample(rddf_sample, d_interval, d_time, ANGLE2D(rddf[i], rddf[i + 1]));
		}
	}
}

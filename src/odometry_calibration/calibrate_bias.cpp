#include <pso.h>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace std;

class Line
{
	public:
		double v;
		double phi;
		double time;
		double gps_x;
		double gps_y;
		double gps_yaw;
		double gps_time;
};


typedef struct
{
	vector<Line> lines;
}PsoData;


PsoData DataReadFromFile;


double
carmen_normalize_theta(double theta)
{
	double multiplier;

	if (theta >= -M_PI && theta < M_PI)
		return theta;

	multiplier = floor(theta / (2*M_PI));
	theta = theta - multiplier*2*M_PI;

	if (theta >= M_PI)
		theta -= 2*M_PI;
	if (theta < -M_PI)
		theta += 2*M_PI;

	return theta;
}


void
read_data(char *filename)
{
	int n;
	char ignored_data[32];
	FILE *f = fopen(filename, "r");

	int first = 1;
	double dist = 0.0;
	double xbef, ybef;

	if (f != NULL)
	{
		while(!feof(f))
		{
			Line l;

			n = fscanf(f, "%s %lf %lf %lf %lf %lf %lf %lf\n", ignored_data,
				&l.v, &l.phi, &l.time, &l.gps_x, &l.gps_y, &l.gps_yaw, &l.gps_time);

			if (n == 8) // this test is only to avoid compiler warning
				DataReadFromFile.lines.push_back(l);

			if (!first)
				dist += sqrt(pow(l.gps_x - xbef, 2) + pow(l.gps_y - ybef, 2));

			xbef = l.gps_x;
			ybef = l.gps_y;
			first = 0;
		}
	}
	else
		exit(printf("File '%s' not found!\n", filename));

	fprintf(stderr, "gps dist: %lf\n", dist);
	fclose(f);
}


void
print_result(double *particle)
{
	uint i;
	double dt, x, y, yaw, v, phi;
	double dt_gps_and_odom, dt_gps_and_odom_acc;
	double x_withoutbias, y_withoutbias, yaw_withoutbias;
	double gps_x, gps_y;
	PsoData *pso_data;

	pso_data = &DataReadFromFile;

	x = 0;
	y = 0;

	x_withoutbias = 0;
	y_withoutbias = 0;

	yaw = particle[4]; // pso_data->lines[0].gps_yaw;
	yaw_withoutbias = particle[4]; // pso_data->lines[0].gps_yaw;

	dt_gps_and_odom = 0;
	dt_gps_and_odom_acc = 0;

	for (i = 1; i < pso_data->lines.size(); i++)
	{
		dt = pso_data->lines[i].time - pso_data->lines[i - 1].time;

		v = pso_data->lines[i].v * particle[0] + particle[1];
		phi = carmen_normalize_theta(pso_data->lines[i].phi * particle[2] + particle[3]);

		x = x + dt * v * cos(yaw);
		y = y + dt * v * sin(yaw);
		yaw = yaw + dt * (v / 2.625 /* L */) * tan(phi);
		yaw = carmen_normalize_theta(yaw);

		x_withoutbias = x_withoutbias + dt * pso_data->lines[i].v * cos(yaw_withoutbias);
		y_withoutbias = y_withoutbias + dt * pso_data->lines[i].v * sin(yaw_withoutbias);
		yaw_withoutbias = yaw_withoutbias + dt * (pso_data->lines[i].v / 2.625 /* L */) * tan(pso_data->lines[i].phi);
		yaw_withoutbias = carmen_normalize_theta(yaw_withoutbias);

		gps_x = pso_data->lines[i].gps_x - pso_data->lines[0].gps_x;
		gps_y = pso_data->lines[i].gps_y - pso_data->lines[0].gps_y;

		dt_gps_and_odom = fabs(pso_data->lines[i].time - pso_data->lines[i].gps_time);
		dt_gps_and_odom_acc += dt_gps_and_odom;

		printf("%lf %lf %lf %lf %lf %lf %lf %lf\n", x, y, gps_x, gps_y, x_withoutbias, y_withoutbias, dt_gps_and_odom, dt_gps_and_odom_acc);
	}
}


double 
fitness(double *particle, void *data, int particle_id __attribute__ ((unused)))
{
	uint i;
	double error;
	double dt, x, y, yaw, v, phi;
	double gps_x, gps_y;
	PsoData *pso_data;

	error = 0;
	pso_data = (PsoData *) data;

	x = 0;
	y = 0;

	// initialize with the gps's yaw to achieve the right alignment
	yaw = particle[4]; // pso_data->lines[0].gps_yaw;
	yaw = carmen_normalize_theta(yaw);

	for (i = 1; i < pso_data->lines.size(); i++)
	{
		dt = pso_data->lines[i].time - pso_data->lines[i - 1].time;

		// v = raw_v * mult_bias + add_bias
		v = pso_data->lines[i].v * particle[0] + particle[1];

		// phi = raw_phi * mult_bias + add_bias
		phi = carmen_normalize_theta(pso_data->lines[i].phi * particle[2] + particle[3]);

		// ackermann prediction
		x = x + dt * v * cos(yaw);
		y = y + dt * v * sin(yaw);
		yaw = yaw + dt * (v / 2.625 /* L */) * tan(phi);
		yaw = carmen_normalize_theta(yaw);

		// translate the starting pose of gps to zero to avoid floating point numerical instability
		gps_x = pso_data->lines[i].gps_x - pso_data->lines[0].gps_x;
		gps_y = pso_data->lines[i].gps_y - pso_data->lines[0].gps_y;

		// add the error
		error += sqrt(pow(x - gps_x, 2) + pow(y - gps_y, 2));
	}

	// pso only maximizes, so we use as fitness the inverse of the error.
	return -(error / (double) pso_data->lines.size());
}


double **
alloc_limits(int dim)
{
	int i;
	double **limits;

	limits = (double **) calloc (dim, sizeof(double*));
	
	for (i = 0; i < dim; i++)
		limits[i] = (double *) calloc (2, sizeof(double));

	return limits;
}


double**
set_limits(int dim)
{
	double **limits;

	limits = alloc_limits(dim);

	// v multiplicative bias
	limits[0][0] = 0.5;
	limits[0][1] = 1.5;

	// v additive bias
	limits[1][0] = -0.00000001;
	limits[1][1] = 0.00000001;

	// phi multiplicative bias
	limits[2][0] = 0.5;
	limits[2][1] = 1.5;

	// phi additive bias
	limits[3][0] = -0.3;
	limits[3][1] = 0.3;

	// initial angle
	limits[4][0] = -3.14;
	limits[4][1] = 3.14;
//	limits[4][0] = 0.62 - 0.05;
//	limits[4][1] = 0.62 + 0.05;

	return limits;
}


int 
main(int argc, char **argv)
{
	double **limits;

	if (argc < 2)
		exit(printf("Use %s <input-data>\n", argv[0]));

	read_data(argv[1]);
	limits = set_limits(5);

	srand(time(NULL));
	srand(rand());

	ParticleSwarmOptimization optimizer(
		fitness, limits, 5, &DataReadFromFile, 400, 1000);

	optimizer.Optimize();

	fprintf(stderr, "bias v: %lf %lf bias phi: %lf %lf initial angle: %lf\n",
		optimizer.GetBestSolution()[0], optimizer.GetBestSolution()[1],
		optimizer.GetBestSolution()[2], optimizer.GetBestSolution()[3],
		optimizer.GetBestSolution()[4]
	);

	fprintf(stderr, "Fitness (MSE): %lf\n", optimizer.GetBestFitness());
	fprintf(stderr, "Fitness (SQRT(MSE)): %lf\n", sqrt(fabs(optimizer.GetBestFitness())));

	// DEBUG: it prints the calibrated odometry, gps and raw odometry
	print_result(optimizer.GetBestSolution());

	return 0;
}



#include <pso.h>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <carmen/carmen.h>
#include <carmen/Gdc_Coord_3d.h>
#include <carmen/Utm_Coord_3d.h>
#include <carmen/Gdc_To_Utm_Converter.h>
#include <carmen/robot_ackerman_messages.h>
#include <carmen/gps_xyz_interface.h>


using namespace std;


#define MAX_LINE_LENGTH (5*4000000)


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


carmen_robot_ackerman_velocity_message read_odometry(FILE *f)
{
	carmen_robot_ackerman_velocity_message m;
	fscanf(f, "%lf %lf %lf", &m.v, &m.phi, &m.timestamp);
	return m;
}


carmen_gps_xyz_message read_gps(FILE *f)
{
	static char dummy[128];

    double lt_dm, lt, lg_dm, lg, sea_level;
    char lt_orientation, lg_orientation;
    int quality, gps_id;

    carmen_gps_xyz_message m;
    memset(&m, 0, sizeof(m));

    fscanf(f, "%d", &gps_id);

    if (1 == gps_id)
    {
    	fscanf(f, "%s", dummy);

    	fscanf(f, "%lf", &lt_dm);
    	fscanf(f, " %c ", &lt_orientation); // read a char ignoring space
    	fscanf(f, "%lf", &lg_dm);
    	fscanf(f, " %c ", &lg_orientation); // read a char ignoring space
    	fscanf(f, "%d", &quality);

    	fscanf(f, "%s", dummy);
    	fscanf(f, "%s", dummy);

    	fscanf(f, "%lf", &sea_level);

    	fscanf(f, "%s", dummy);
    	fscanf(f, "%s", dummy);
    	fscanf(f, "%s", dummy);
    	fscanf(f, "%s", dummy);

    	fscanf(f, "%lf", &m.timestamp);

        lt = carmen_global_convert_degmin_to_double(lt_dm);
        lg = carmen_global_convert_degmin_to_double(lg_dm);

        // verify the latitude and longitude orientations
        if ('S' == lt_orientation) lt = -lt;
        if ('W' == lg_orientation) lg = -lg;

        // convert to x and y coordinates
        Gdc_Coord_3d gdc = Gdc_Coord_3d(lt, lg, sea_level);

        // Transformando o z utilizando como altitude a altitude mesmo - que esta vindo como zero
        Utm_Coord_3d utm;
        Gdc_To_Utm_Converter::Init();
        Gdc_To_Utm_Converter::Convert(gdc , utm);

        m.x = utm.y;
        m.y = -utm.x;
    }

    return m;
}


void
process_gps(carmen_gps_xyz_message &m, vector<carmen_robot_ackerman_velocity_message> &odoms)
{
	if (odoms.size() == 0 || m.timestamp == 0)
		return;

	int near = 0;
	for (size_t i = 0; i < odoms.size(); i++)
	{
		if (fabs(m.timestamp - odoms[i].timestamp) < fabs(m.timestamp - odoms[near].timestamp))
			near = i;
	}

	Line l;

	l.v = odoms[near].v;
	l.phi = odoms[near].phi;
	l.time = odoms[near].timestamp;
	l.gps_x = m.x;
	l.gps_y = m.y;
	l.gps_yaw = 0.;
	l.gps_time = m.timestamp;

	// DEBUG:
	//printf("%lf %lf %lf %lf %lf %lf\n", l.v, l.phi, l.time, l.gps_x, l.gps_y, l.gps_time);

	DataReadFromFile.lines.push_back(l);
	odoms.clear();
}


void
read_data(char *filename)
{
	printf("Reading log...\n");
	static char tag[256];
	static char line[MAX_LINE_LENGTH];

	FILE *f = fopen(filename, "r");

	vector<carmen_robot_ackerman_velocity_message> odoms;

	if (f != NULL)
	{
		while(!feof(f))
		{
			fscanf(f, "\n%s", tag);

			if (!strcmp(tag, "NMEAGGA"))
			{
				carmen_gps_xyz_message m = read_gps(f);
				process_gps(m, odoms);
			}

			else if (!strcmp(tag, "ROBOTVELOCITY_ACK"))
			{
				carmen_robot_ackerman_velocity_message m = read_odometry(f);
				odoms.push_back(m);
			}
			else
				fscanf(f, "%[^\n]\n", line);
		}
	}
	else
		exit(printf("File '%s' not found!\n", filename));

	fclose(f);
	printf("Done.\n");
}


double
estimate_theta(PsoData *pso_data, int id)
{
	for (uint i = id; i < pso_data->lines.size(); i++)
	{
		double dy = pso_data->lines[i].gps_y - pso_data->lines[id].gps_y;
		double dx = pso_data->lines[i].gps_x - pso_data->lines[id].gps_x;

		double d = sqrt(pow(dx, 2) +
						pow(dy, 2));

		if (d > 10)
			return atan2(dy, dx);
	}

	exit(printf("Error: unable of finding a reasonable initial angle."));
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

	yaw = yaw_withoutbias = particle[4]; // estimate_theta(pso_data, 0);
	fprintf(stderr, "Initial angle: %lf\n", yaw);

	dt_gps_and_odom = 0;
	dt_gps_and_odom_acc = 0;

	for (i = 1; i < pso_data->lines.size(); i++)
	{
		dt = pso_data->lines[i].time - pso_data->lines[i - 1].time;

		// for the case in which we are optimizing a merged log.
		if (dt > 6000 || dt < 0)
		{
			//fprintf(stderr, "Initiating a new log at message %d with dt: %lf\n", i, dt);
			x = pso_data->lines[i].gps_x - pso_data->lines[0].gps_x;
			y = pso_data->lines[i].gps_y - pso_data->lines[0].gps_y;
			yaw = yaw_withoutbias = particle[4]; //estimate_theta(pso_data, i);
			fprintf(stderr, "Reseting initial angle: %lf\n", yaw);
			continue;
		}

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
fitness(double *particle, void *data)
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

	yaw = particle[4]; // estimate_theta(pso_data, 0);

	for (i = 1; i < pso_data->lines.size(); i++)
	{
		dt = pso_data->lines[i].time - pso_data->lines[i - 1].time;

		// for the case in which we are optimizing a merged log.
		if (dt > 6000 || dt < 0)
		{
			//fprintf(stderr, "Initiating a new log at message %d with dt: %lf\n", i, dt);
			x = pso_data->lines[i].gps_x - pso_data->lines[0].gps_x;
			y = pso_data->lines[i].gps_y - pso_data->lines[0].gps_y;
			yaw = particle[4]; // estimate_theta(pso_data, i);
			continue;
		}

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

	// Initial angle
	limits[4][0] = -M_PI;
	limits[4][1] = M_PI;

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
		fitness, limits, 5, &DataReadFromFile, 1000, 100);

	optimizer.Optimize();

	fprintf(stderr, "bias v: %lf %lf bias phi: %lf %lf Initial Angle: %lf\n",
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


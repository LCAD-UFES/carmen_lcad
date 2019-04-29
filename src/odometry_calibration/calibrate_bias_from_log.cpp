
#include <pso.h>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>

#include <carmen/carmen.h>
#include <carmen/Gdc_Coord_3d.h>
#include <carmen/Utm_Coord_3d.h>
#include <carmen/Gdc_To_Utm_Converter.h>
#include <carmen/robot_ackerman_messages.h>
#include <carmen/gps_xyz_interface.h>


using namespace std;


#define MAX_LINE_LENGTH (5*4000000)
#define INITIAL_LOG_LINE 1
#define MAX_LOG_LINES 50000

#define L 2.85
#define MAX_STEERING_ANGLE 0.6
#define NUM_PHI_SPLINE_KNOTS 3

gsl_interp_accel *acc;
gsl_spline *phi_spline;


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


carmen_robot_ackerman_velocity_message
read_odometry(FILE *f)
{
	carmen_robot_ackerman_velocity_message m;
	fscanf(f, "%lf %lf %lf", &m.v, &m.phi, &m.timestamp);
	return m;
}


carmen_gps_xyz_message
read_gps(FILE *f, int gps_to_use)
{
	static char dummy[128];

	double lt_dm, lt, lg_dm, lg, sea_level;
	char lt_orientation, lg_orientation;
	int quality, gps_id;

	carmen_gps_xyz_message m;
	memset(&m, 0, sizeof(m));

	fscanf(f, "%d", &gps_id);

	if (gps_to_use == gps_id)
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
read_data(char *filename, int gps_to_use)
{
	printf("Reading log...\n");
	static char tag[256];
	static char line[MAX_LINE_LENGTH];

	FILE *f = fopen(filename, "r");

	if (f != NULL)
	{
		for (int i = 0; i < INITIAL_LOG_LINE; i++)
		{
			if (!fgets(line, MAX_LINE_LENGTH - 1, f))
			{
				printf("EOF in log at read_data(). The log has less than %d lines\n", INITIAL_LOG_LINE);
				exit(1);
			}
		}

		vector<carmen_robot_ackerman_velocity_message> odoms;

		int num_lines = 0;
		while(!feof(f) && num_lines < MAX_LOG_LINES)
		{
			fscanf(f, "\n%s", tag);

			if (!strcmp(tag, "NMEAGGA"))
			{
				carmen_gps_xyz_message m = read_gps(f, gps_to_use);
				process_gps(m, odoms);
			}
			else if (!strcmp(tag, "ROBOTVELOCITY_ACK"))
			{
				carmen_robot_ackerman_velocity_message m = read_odometry(f);
				odoms.push_back(m);
			}
			else
				fscanf(f, "%[^\n]\n", line);

			num_lines++;
		}
	}
	else
		exit(printf("File '%s' not found!\n", filename));

	fclose(f);
	printf("Done.\n");

	if (DataReadFromFile.lines.size() <= 0)
		exit(printf("Error: Unable to load data from log '%s'. Are you sure you logged gps '%d'?\n", filename, gps_to_use));
}


double
estimate_theta(PsoData *pso_data, int id)
{
	/**
    Estimates theta using the first GPS positions ahead. 
    This method fails if the car is moving backwards in the beginning of the log, 
        or if it starts in a curve.
	 **/
	for (uint i = id; i < pso_data->lines.size(); i++)
	{
		double dy = pso_data->lines[i].gps_y - pso_data->lines[id].gps_y;
		double dx = pso_data->lines[i].gps_x - pso_data->lines[id].gps_x;

		double d = sqrt(pow(dx, 2) + pow(dy, 2));

		if (d > 5)
			return atan2(dy, dx);
	}

	exit(printf("Error: unable to find a reasonable initial angle."));
}


//static double
//factor(double x)
//{
//	double value = 1.0 / (1.0 + fabs(x * x));
//	return (value);
//}


void
ackerman_model(double &x, double &y, double &yaw, double v, double phi, double dt)
{
	double new_phi = phi / (1.0 + v * v * 0.000); // underster IARA
//	new_phi = new_phi * factor(6.0 * new_phi);

	x = x + dt * v * cos(yaw);
	y = y + dt * v * sin(yaw);
	yaw = yaw + dt * (v / L) * tan(new_phi);
	yaw = carmen_normalize_theta(yaw);
}


void
print_phi_spline(gsl_spline *phi_spline, gsl_interp_accel *acc, double max_phi, bool display_phi_profile)
{
	if (!display_phi_profile)
		return;

	FILE *path_file = fopen("spline_plot.txt" , "w");

	for (double phi = -max_phi; phi < max_phi; phi += max_phi / 100.0)
	{
		double new_phi;
		if (phi < 0.0)
			new_phi = -gsl_spline_eval(phi_spline, -phi, acc);
		else
			new_phi = gsl_spline_eval(phi_spline, phi, acc);

		fprintf(path_file, "%f %f\n", phi, new_phi);
	}

	fclose(path_file);
}


void
compute_phi_spline(double particle_a, double particle_b)
{
//	double k1 = (1.0 / 3.0) * MAX_STEERING_ANGLE + particle_a;
//	double k2 = (2.0 / 3.0) * MAX_STEERING_ANGLE + particle_b;
//
//	double knots_x[NUM_PHI_SPLINE_KNOTS] = {0.0, (1.0 / 3.0) * MAX_STEERING_ANGLE, (2.0 / 3.0) * MAX_STEERING_ANGLE, MAX_STEERING_ANGLE};
//	double knots_y[NUM_PHI_SPLINE_KNOTS] = {0.0, k1, k2, MAX_STEERING_ANGLE};

	double k1 = (1.0 / 2.0 + particle_a) * MAX_STEERING_ANGLE + particle_b;

	double knots_x[NUM_PHI_SPLINE_KNOTS] = {0.0, (1.0 / 2.0 + particle_a) * MAX_STEERING_ANGLE, MAX_STEERING_ANGLE};
	double knots_y[NUM_PHI_SPLINE_KNOTS] = {0.0, k1, MAX_STEERING_ANGLE};

	gsl_spline_init(phi_spline, knots_x, knots_y, NUM_PHI_SPLINE_KNOTS);
}


double
compute_optimized_odometry_pose(double &x, double &y, double &yaw, double *particle, PsoData *pso_data, int i)
{
	double dt = pso_data->lines[i].gps_time - pso_data->lines[i - 1].gps_time;
	// v = raw_v * mult_bias + add_bias
	double v = pso_data->lines[i].v * particle[0] + particle[1];

	// phi = raw_phi + add_bias
	double phi = pso_data->lines[i].phi + particle[3];
	if (phi < 0.0)
		phi = -gsl_spline_eval(phi_spline, -phi, acc);
	else
		phi = gsl_spline_eval(phi_spline, phi, acc);

	// phi = raw_phi * mult_bias
	phi = phi * particle[2];

	ackerman_model(x, y, yaw, v, phi, dt);

	return (v);
}


void
print_result(double *particle, FILE *f_report)
{
	PsoData *pso_data = &DataReadFromFile;

	double x = 0.0;
	double y = 0.0;
	double yaw = particle[4];
	double yaw_withoutbias = yaw;
	//yaw = yaw_withoutbias = estimate_theta(pso_data, 0);

	double x_withoutbias = 0.0;
	double y_withoutbias = 0.0;

	compute_phi_spline(particle[5], particle[6]);

	fprintf(f_report, "Initial angle: %lf\n", yaw);
//	fprintf(stderr, "Initial angle: %lf\n", yaw);

	double dt_gps_and_odom_acc = 0.0;

	for (uint i = 1; i < pso_data->lines.size(); i++)
	{
		double v = compute_optimized_odometry_pose(x, y, yaw, particle, pso_data, i);

		if (v > 1.0)
		{
			double dt = pso_data->lines[i].gps_time - pso_data->lines[i - 1].gps_time;
			ackerman_model(x_withoutbias, y_withoutbias, yaw_withoutbias, pso_data->lines[i].v, pso_data->lines[i].phi, dt);

			double gps_x = pso_data->lines[i].gps_x - pso_data->lines[0].gps_x;
			double gps_y = pso_data->lines[i].gps_y - pso_data->lines[0].gps_y;

			double dt_gps_and_odom = fabs(pso_data->lines[i].time - pso_data->lines[i].gps_time);
			dt_gps_and_odom_acc += dt_gps_and_odom;

			fprintf(f_report, "DATA %lf %lf %lf %lf %lf %lf %lf %lf\n", x, y, gps_x, gps_y, x_withoutbias, y_withoutbias, dt_gps_and_odom, dt_gps_and_odom_acc);
		}
	}
}


double 
fitness(double *particle, void *data)
{
	PsoData *pso_data = (PsoData *) data;

	double x = 0.0;
	double y = 0.0;
	double yaw = particle[4];
	//yaw = estimate_theta(pso_data, 0);

	compute_phi_spline(particle[5], particle[6]);

	double error = 0.0;
	double count = 0;
	for (uint i = 1; i < pso_data->lines.size(); i++)
	{
		double v = compute_optimized_odometry_pose(x, y, yaw, particle, pso_data, i);
		if (v > 1.0)
		{
			// translate the starting pose of gps to zero to avoid floating point numerical instability
			double gps_x = pso_data->lines[i].gps_x - pso_data->lines[0].gps_x;
			double gps_y = pso_data->lines[i].gps_y - pso_data->lines[0].gps_y;

			// add the error
			error += sqrt(pow(x - gps_x, 2.0) + pow(y - gps_y, 2.0));

			// reinforce consistency between heading direction and heading estimated using gps
			double gps_yaw = atan2(pso_data->lines[i].gps_y - pso_data->lines[i - 1].gps_y,
								   pso_data->lines[i].gps_x - pso_data->lines[i - 1].gps_x);

//			error += 200.0 * fabs(carmen_normalize_theta(gps_yaw - yaw));
			count += 1.0;
		}
	}

	// pso only maximizes, so we use as fitness the inverse of the error.
	return (-error / count);
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
	//limits[0][0] = 0.95; //0.5;
	//limits[0][1] = 1.05; //1.5;
	limits[0][0] = 0.979999;
	limits[0][1] = 1.3001;

	// v additive bias
	limits[1][0] = -0.00000001;
	limits[1][1] = 0.00000001;

	// phi multiplicative bias
	limits[2][0] = 0.55;
	limits[2][1] = 2.5;

	// phi additive bias
	limits[3][0] = -carmen_degrees_to_radians(2.);
	limits[3][1] = carmen_degrees_to_radians(2.);

	// Initial angle
	limits[4][0] = -M_PI;
	limits[4][1] = M_PI;

	// k1 of phi spline
	limits[5][0] = -0.3;
	limits[5][1] = +0.3;

	// k1 of phi spline
	limits[6][0] = -0.15;
	limits[6][1] = +0.15;

	return (limits);
}


void
plot_graph(double *particle)
{
	static bool first_time = true;
	static FILE *gnuplot_pipe;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipe = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipe, "set size square\n");
		fprintf(gnuplot_pipe, "set size ratio -1\n");
//		fprintf(gnuplot_pipe, "set xrange [0:70]\n");
//		fprintf(gnuplot_pipe, "set yrange [-10:10]\n");
//		fprintf(gnuplot_pipe, "set xlabel 'senconds'\n");
//		fprintf(gnuplot_pipe, "set ylabel 'effort'\n");
	}

	FILE *gnuplot_data_file = fopen("gnuplot_data.txt", "w");

	print_result(particle, gnuplot_data_file);

	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_data.txt' u 2:3 w l,"
			"'./gnuplot_data.txt' u 4:5 w l\n");

	fflush(gnuplot_pipe);
}


int 
main(int argc, char **argv)
{
	double **limits;

	if (argc < 4)
		exit(printf("Use %s <log_path> <output_calibration_file> <output_report_file> <gps_to_use: default: 1>\n", argv[0]));

	int gps_to_use = 1;

	if (argc == 5)
		gps_to_use = atoi(argv[4]);

	read_data(argv[1], gps_to_use);

	acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	phi_spline = gsl_spline_alloc(type, NUM_PHI_SPLINE_KNOTS);

	limits = set_limits(7);

	FILE *f_calibration = fopen(argv[2], "w");
	FILE *f_report = fopen(argv[3], "w");

	if (f_calibration == NULL) exit(printf("Error: unable to open file '%s'\n", argv[2]));
	if (f_report == NULL) exit(printf("Error: unable to open file '%s'\n", argv[3]));

	srand(time(NULL));
	srand(rand());

	ParticleSwarmOptimization optimizer(fitness, limits, 7, &DataReadFromFile, 500, 200);

	optimizer.Optimize(plot_graph);

	fprintf(f_calibration, "v (multiplier bias): (%lf %lf),  phi (multiplier bias): (%lf %lf),  Initial Angle: %lf,  k1: %lf,  k2: %lf\n",
	        optimizer.GetBestSolution()[0], optimizer.GetBestSolution()[1],
	        optimizer.GetBestSolution()[2], optimizer.GetBestSolution()[3],
	        optimizer.GetBestSolution()[4],
	        optimizer.GetBestSolution()[5],
	        optimizer.GetBestSolution()[6]
	);

	fprintf(stderr, "v (multiplier bias): (%lf %lf),  phi (multiplier bias): (%lf %lf),  Initial Angle: %lf,  k1: %lf,  k2: %lf\n",
	        optimizer.GetBestSolution()[0], optimizer.GetBestSolution()[1],
	        optimizer.GetBestSolution()[2], optimizer.GetBestSolution()[3],
	        optimizer.GetBestSolution()[4],
	        optimizer.GetBestSolution()[5],
	        optimizer.GetBestSolution()[6]
	);

	fprintf(f_report, "Fitness (MSE): %lf\n", optimizer.GetBestFitness());
	fprintf(f_report, "Fitness (SQRT(MSE)): %lf\n", sqrt(fabs(optimizer.GetBestFitness())));

	fprintf(stderr, "Fitness (MSE): %lf\n", optimizer.GetBestFitness());
	fprintf(stderr, "Fitness (SQRT(MSE)): %lf\n", sqrt(fabs(optimizer.GetBestFitness())));

	// DEBUG: it prints the calibrated odometry, gps and raw odometry
	print_result(optimizer.GetBestSolution(), f_report);

	print_phi_spline(phi_spline, acc, MAX_STEERING_ANGLE, true);

	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);

	return (0);
}


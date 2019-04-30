
#include <pso.h>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>

#include <carmen/carmen.h>
#include <carmen/Gdc_Coord_3d.h>
#include <carmen/Utm_Coord_3d.h>
#include <carmen/Gdc_To_Utm_Converter.h>
#include <carmen/robot_ackerman_messages.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/command_line.h>
#include <carmen/carmen_param_file.h>
#include <carmen/util_io.h>
#include <carmen/tf.h>

using namespace std;
using namespace tf;

#define MAX_LINE_LENGTH (5*4000000)
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
	double distance_between_front_and_rear_axles;
	double max_steering_angle;
	Transform gps2car;
}PsoData;


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
process_gps(carmen_gps_xyz_message &m, vector<carmen_robot_ackerman_velocity_message> &odoms, PsoData *pso_data)
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

	pso_data->lines.push_back(l);
	odoms.clear();
}


void
skip_first_lines(FILE *f, int initial_log_line, char line[])
{
	for (int i = 0; i < initial_log_line; i++)
	{
		if (!fgets(line, MAX_LINE_LENGTH - 1, f))
		{
			printf("EOF in log at read_data(). The log has less than %d lines\n", initial_log_line);
			exit(1);
		}
	}
}


void
read_data(const char *filename, int gps_to_use, int initial_log_line, int max_log_lines, PsoData *pso_data)
{
	static char tag[256];
	static char line[MAX_LINE_LENGTH];

	int num_lines;
	vector<carmen_robot_ackerman_velocity_message> odoms;

	printf("Reading log...\n");
	FILE *f = safe_fopen(filename, "r");

	if (max_log_lines < 0)
		max_log_lines = INT_MAX;

	skip_first_lines(f, initial_log_line, line);

	num_lines = 0;
	while(!feof(f) && num_lines < max_log_lines)
	{
		fscanf(f, "\n%s", tag);

		if (!strcmp(tag, "NMEAGGA"))
		{
			carmen_gps_xyz_message m = read_gps(f, gps_to_use);
			process_gps(m, odoms, pso_data);
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

	fclose(f);
	printf("Done.\n");

	if (pso_data->lines.size() <= 0)
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
ackerman_model(double &x, double &y, double &yaw, double v, double phi, double dt, double L)
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
compute_phi_spline(double particle_a, double particle_b, double max_steering_angle)
{
//	double k1 = (1.0 / 3.0) * MAX_STEERING_ANGLE + particle_a;
//	double k2 = (2.0 / 3.0) * MAX_STEERING_ANGLE + particle_b;
//
//	double knots_x[NUM_PHI_SPLINE_KNOTS] = {0.0, (1.0 / 3.0) * MAX_STEERING_ANGLE, (2.0 / 3.0) * MAX_STEERING_ANGLE, MAX_STEERING_ANGLE};
//	double knots_y[NUM_PHI_SPLINE_KNOTS] = {0.0, k1, k2, MAX_STEERING_ANGLE};

	double k1 = (1.0 / 2.0 + particle_a) * max_steering_angle + particle_b;

	double knots_x[NUM_PHI_SPLINE_KNOTS] = {0.0, (1.0 / 2.0 + particle_a) * max_steering_angle, max_steering_angle};
	double knots_y[NUM_PHI_SPLINE_KNOTS] = {0.0, k1, max_steering_angle};

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

	ackerman_model(x, y, yaw, v, phi, dt, pso_data->distance_between_front_and_rear_axles);

	return (v);
}


void
transform_gps_to_car(double *gps_x, double *gps_y, Transform &gps2car)
{
	Vector3 p_car;
	Vector3 p_gps;

	p_gps = Vector3(*gps_x, *gps_y, 0);
	p_car = gps2car * p_gps;

	*gps_x = p_car.getX();
	*gps_y = p_car.getY();
}


void
print_result(double *particle, FILE *f_report, PsoData *pso_data)
{
	double x = 0.0;
	double y = 0.0;
	double yaw = particle[4];
	double yaw_withoutbias = yaw;
	//yaw = yaw_withoutbias = estimate_theta(pso_data, 0);

	double x_withoutbias = 0.0;
	double y_withoutbias = 0.0;

	compute_phi_spline(particle[5], particle[6], pso_data->max_steering_angle);

	fprintf(f_report, "Initial angle: %lf\n", yaw);
//	fprintf(stderr, "Initial angle: %lf\n", yaw);

	double dt_gps_and_odom_acc = 0.0;

	for (uint i = 1; i < pso_data->lines.size(); i++)
	{
		double v = compute_optimized_odometry_pose(x, y, yaw, particle, pso_data, i);

		if (v > 1.0)
		{
			double dt = pso_data->lines[i].gps_time - pso_data->lines[i - 1].gps_time;
			ackerman_model(x_withoutbias, y_withoutbias, yaw_withoutbias, pso_data->lines[i].v, pso_data->lines[i].phi, dt,
										 pso_data->distance_between_front_and_rear_axles);

			double gps_x = pso_data->lines[i].gps_x - pso_data->lines[0].gps_x;
			double gps_y = pso_data->lines[i].gps_y - pso_data->lines[0].gps_y;

			transform_gps_to_car(&gps_x, &gps_y, pso_data->gps2car);

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

	compute_phi_spline(particle[5], particle[6], pso_data->max_steering_angle);

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

			// Uncomment the printfs for visualizing the result of the gps data transformation:
			//printf("Before: %.2lf %.2lf ", gps_x, gps_y);
			transform_gps_to_car(&gps_x, &gps_y, pso_data->gps2car);
			//printf("After: %.2lf %.2lf\n", gps_x, gps_y);

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
plot_graph(ParticleSwarmOptimization *optimizer, void *data)
{
	static bool first_time = true;
	static FILE *gnuplot_pipe;
	double *particle;
	PsoData *pso_data;

	pso_data = (PsoData *) data;
	particle = optimizer->GetBestSolution();

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
	print_result(particle, gnuplot_data_file, pso_data);
	fclose(gnuplot_data_file);
	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_data.txt' u 2:3 w l,"
			"'./gnuplot_data.txt' u 4:5 w l\n");

	fflush(gnuplot_pipe);
}


void
print_optimization_report(FILE* f_calibration, FILE* f_report, ParticleSwarmOptimization &optimizer)
{
	fprintf(f_calibration,
			"v (multiplier bias): (%lf %lf),  phi (multiplier bias): (%lf %lf),  Initial Angle: %lf,  k1: %lf,  k2: %lf\n",
			optimizer.GetBestSolution()[0], optimizer.GetBestSolution()[1],
			optimizer.GetBestSolution()[2], optimizer.GetBestSolution()[3],
			optimizer.GetBestSolution()[4], optimizer.GetBestSolution()[5],
			optimizer.GetBestSolution()[6]);

	fprintf(stderr,
			"v (multiplier bias): (%lf %lf),  phi (multiplier bias): (%lf %lf),  Initial Angle: %lf,  k1: %lf,  k2: %lf\n",
			optimizer.GetBestSolution()[0], optimizer.GetBestSolution()[1],
			optimizer.GetBestSolution()[2], optimizer.GetBestSolution()[3],
			optimizer.GetBestSolution()[4], optimizer.GetBestSolution()[5],
			optimizer.GetBestSolution()[6]);

	fprintf(f_report, "Fitness (MSE): %lf\n", optimizer.GetBestFitness());
	fprintf(f_report, "Fitness (SQRT(MSE)): %lf\n", sqrt(fabs(optimizer.GetBestFitness())));
	fprintf(stderr, "Fitness (MSE): %lf\n", optimizer.GetBestFitness());
	fprintf(stderr, "Fitness (SQRT(MSE)): %lf\n", sqrt(fabs(optimizer.GetBestFitness())));
}


Transform
get_gps_to_car_transform(CarmenParamFile *params, int gps_to_use, int board_to_use)
{
	char param_name[1024];
	double gps_x, gps_y, gps_z, gps_roll, gps_pitch, gps_yaw;
	double board_x, board_y, board_z, board_roll, board_pitch, board_yaw;
	Transform board2car, gps2board;

	if (gps_to_use == 1)
	{
		fprintf(stderr, "** Warning: We're assuming that the chosen GPS is the one from the sensor_board.\n");
		gps_x = params->get<double>("gps_nmea_x");
		gps_y = params->get<double>("gps_nmea_y");
		gps_z = params->get<double>("gps_nmea_z");
		gps_yaw = params->get<double>("gps_nmea_yaw");
		gps_pitch = params->get<double>("gps_nmea_pitch");
		gps_roll = params->get<double>("gps_nmea_roll");
	}
	else
	{
		sprintf(param_name, "gps_nmea_%d_x", gps_to_use);
		gps_x = params->get<double>(param_name);

		sprintf(param_name, "gps_nmea_%d_y", gps_to_use);
		gps_y = params->get<double>(param_name);

		sprintf(param_name, "gps_nmea_%d_z", gps_to_use);
		gps_z = params->get<double>(param_name);

		sprintf(param_name, "gps_nmea_%d_yaw", gps_to_use);
		gps_yaw = params->get<double>(param_name);

		sprintf(param_name, "gps_nmea_%d_pitch", gps_to_use);
		gps_pitch = params->get<double>(param_name);

		sprintf(param_name, "gps_nmea_%d_roll", gps_to_use);
		gps_roll = params->get<double>(param_name);
		}

	sprintf(param_name, "sensor_board_%d_x", board_to_use);
	board_x = params->get<double>(param_name);

	sprintf(param_name, "sensor_board_%d_y", board_to_use);
	board_y = params->get<double>(param_name);

	sprintf(param_name, "sensor_board_%d_z", board_to_use);
	board_z = params->get<double>(param_name);

	sprintf(param_name, "sensor_board_%d_yaw", board_to_use);
	board_yaw = params->get<double>(param_name);

	sprintf(param_name, "sensor_board_%d_pitch", board_to_use);
	board_pitch = params->get<double>(param_name);

	sprintf(param_name, "sensor_board_%d_roll", board_to_use);
	board_roll = params->get<double>(param_name);

	gps2board = Transform(Quaternion(gps_yaw, gps_pitch, gps_roll), Vector3(gps_x, gps_y, gps_z));
	board2car = Transform(Quaternion(board_yaw, board_pitch, board_roll), Vector3(board_x, board_y, board_z));

	return (board2car * gps2board);
}


void
define_and_parse_args(int argc, char **argv, CommandLineArguments *args)
{
	args->add_positional<string>("log_path", "Path to a log");
	args->add_positional<string>("param_file", "Path to a file containing system parameters");
	args->add_positional<string>("output_calibration", "Path to an output calibration file");
	args->add_positional<string>("output_poses", "Path to a file in which poses will be saved for debug");
	args->add<int>("gps_to_use", "Id of the gps that will be used for the calibration", 1);
	args->add<int>("board_to_use", "Id of the sensor board that will be used for the calibration", 1);
	args->add<int>("n_particles,n", "Number of particles", 500);
	args->add<int>("n_iterations,i", "Number of iterations", 300);
	args->add<int>("initial_log_line,l", "Number of lines to skip in the beggining of the log file", 1);
	args->add<int>("max_log_lines,m", "Maximum number of lines to read from the log file", -1);
	args->save_config_file("odom_calib_config.txt");
	args->parse(argc, argv);
}


int 
main(int argc, char **argv)
{
	int n_params;
	double **limits;
	CommandLineArguments args;
	PsoData pso_data;
	CarmenParamFile *params;

	define_and_parse_args(argc, argv, &args);

	int gps_to_use = args.get<int>("gps_to_use");
	int board_to_use = args.get<int>("board_to_use");

	read_data(args.get<string>("log_path").c_str(), gps_to_use,
						args.get<int>("initial_log_line"),
						args.get<int>("max_log_lines"),
						&pso_data);

	params = new CarmenParamFile(args.get<string>("param_file").c_str());

	pso_data.max_steering_angle = params->get<double>("robot_max_steering_angle");
	pso_data.distance_between_front_and_rear_axles = params->get<double>("robot_distance_between_front_and_rear_axles");
	pso_data.gps2car = get_gps_to_car_transform(params, gps_to_use, board_to_use);

	acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	phi_spline = gsl_spline_alloc(type, NUM_PHI_SPLINE_KNOTS);

	n_params = 7;
	limits = set_limits(n_params);

	FILE *f_calibration = safe_fopen(args.get<string>("output_calibration").c_str(), "w");
	FILE *f_report = safe_fopen(args.get<string>("output_poses").c_str(), "w");

	srand(time(NULL));
	srand(rand()); // ??

	ParticleSwarmOptimization optimizer(fitness, limits, n_params, &pso_data,
																			args.get<int>("n_particles"),
																			args.get<int>("n_iterations"));

	optimizer.Optimize(plot_graph);

	print_optimization_report(f_calibration, f_report, optimizer);
	// DEBUG: it prints the calibrated odometry, gps and raw odometry
	print_result(optimizer.GetBestSolution(), f_report, &pso_data);
	print_phi_spline(phi_spline, acc, pso_data.max_steering_angle, true);

	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);
	delete(params);

	return (0);
}


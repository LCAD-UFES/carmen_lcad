
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

#define MAX_LINE_LENGTH (5 * 4000000)
#define NUM_PHI_SPLINE_KNOTS 3
#define MIN_VELOCITY 0.03

int gps_to_use;
int board_to_use;
int use_non_linear_phi;
int n_params;
double **limits;

gsl_interp_accel *acc;

FILE *gnuplot_pipe;

#define GPS_LATENCY 0.17


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
	double opt_odom_x;
	double opt_odom_y;
	double opt_odom_yaw;
	bool opt_odom_valid;
};


typedef struct
{
	double velodyne_timestamp;
	double odometry_timestamp;
	double v;
	double phi;
} VelodyneAndOdom;


typedef struct
{
	vector<Line> lines;
	vector<VelodyneAndOdom> velodyne_data;
	double distance_between_front_and_rear_axles;
	double max_steering_angle;
	Transformer **tf_transformer;
	gsl_spline **phi_spline;
	string sensor_board_name;
	string gps_name;
	int view_active;
} PsoData;


carmen_robot_ackerman_velocity_message
read_odometry(FILE *f)
{
	carmen_robot_ackerman_velocity_message m;
	fscanf(f, "%lf %lf %lf", &m.v, &m.phi, &m.timestamp);
	return (m);
}


VelodyneAndOdom
read_velodyne_data(FILE *f)
{
	VelodyneAndOdom velodyne_data;
	char scan_file_name[2048];
	int num_shots;

	memset(&velodyne_data, 0, sizeof(velodyne_data));
	fscanf(f, "%s %d %lf", scan_file_name, &num_shots, &(velodyne_data.velodyne_timestamp));

	return (velodyne_data);
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

		m.timestamp = m.timestamp - GPS_LATENCY;
	}

	return m;
}


size_t
search_for_odom_with_nearest_timestamp(vector<carmen_robot_ackerman_velocity_message> &odoms, double reference_timestamp)
{
	size_t i = odoms.size() - 1;
	size_t near = i;

	double smaler_time_distance = reference_timestamp;

	do
	{
		double time_distance = fabs(reference_timestamp - odoms[i].timestamp);

		if (time_distance < smaler_time_distance)
		{
			near = i;
			smaler_time_distance = time_distance;
		}

		if (time_distance > 5.0)
			break;

		i--;
	} while (i != 0);

	return near;
}


void
process_gps(carmen_gps_xyz_message &m, vector<carmen_robot_ackerman_velocity_message> &odoms, PsoData *pso_data)
{
	if (odoms.size() == 0 || m.timestamp == 0)
		return;

	size_t near = search_for_odom_with_nearest_timestamp(odoms, m.timestamp);

	Line l;

	l.v = odoms[near].v;
	l.phi = odoms[near].phi;
	l.time = odoms[near].timestamp;
	l.gps_x = m.x;
	l.gps_y = m.y;
	l.gps_yaw = 0.;
	l.gps_time = m.timestamp;

	pso_data->lines.push_back(l);
}


void
process_velodyne_data(PsoData *pso_data, vector<carmen_robot_ackerman_velocity_message> &odoms, VelodyneAndOdom velodyne_data)
{
	if (odoms.size() == 0 || velodyne_data.velodyne_timestamp == 0)
		return;

	size_t near = search_for_odom_with_nearest_timestamp(odoms, velodyne_data.velodyne_timestamp);

	velodyne_data.v = odoms[near].v;
	velodyne_data.phi = odoms[near].phi;
	velodyne_data.odometry_timestamp = odoms[near].timestamp;

	pso_data->velodyne_data.push_back(velodyne_data);
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
		else if (!strcmp(tag, "VELODYNE_PARTIAL_SCAN_IN_FILE"))
		{
			VelodyneAndOdom velodyne_data = read_velodyne_data(f);
			process_velodyne_data(pso_data, odoms, velodyne_data);
		}

		fscanf(f, "%[^\n]\n", line);

		num_lines++;
	}

	fclose(f);
	printf("Done.\n");

	if (pso_data->lines.size() <= 0)
		exit(printf("Error: Unable to load data from log '%s'. Are you sure you logged gps '%d'?\n", filename, gps_to_use));
}


Transform
read_object_pose_by_name_from_carmen_ini(CarmenParamFile *params, string object_name)
{
	double x, y, z, roll, pitch, yaw;

	x = params->get<double>(object_name + "_x");
	y = params->get<double>(object_name + "_y");
	z = params->get<double>(object_name + "_z");
	roll = params->get<double>(object_name + "_roll");
	pitch = params->get<double>(object_name + "_pitch");
	yaw = params->get<double>(object_name + "_yaw");

	return Transform(Quaternion(yaw, pitch, roll), Vector3(x, y, z));
}


void
initialize_car_objects_poses_in_transformer(CarmenParamFile *params, PsoData *pso_data, int gps_to_use, int board_to_use, int particle_id)
{
	char gps_name[128];
	char board_name[128];

	sprintf(gps_name, "gps_nmea_%d", gps_to_use);
	sprintf(board_name, "sensor_board_%d", board_to_use);

	pso_data->tf_transformer[particle_id]->setTransform(tf::StampedTransform(read_object_pose_by_name_from_carmen_ini(params, board_name), tf::Time(0), "/car", "/board"));
	pso_data->tf_transformer[particle_id]->setTransform(tf::StampedTransform(read_object_pose_by_name_from_carmen_ini(params, gps_name), tf::Time(0), "/board", "/gps"));

	tf::StampedTransform a_to_b;
	pso_data->tf_transformer[particle_id]->lookupTransform("/car", "/board", tf::Time(0), a_to_b);
//	printf("board with respect to car: x: %lf, y: %lf, z: %lf\n", a_to_b.getOrigin().x(), a_to_b.getOrigin().y(), a_to_b.getOrigin().z());
	pso_data->tf_transformer[particle_id]->lookupTransform("/board", "/gps", tf::Time(0), a_to_b);
//	printf("gps with respect to board: x: %lf, y: %lf, z: %lf\n", a_to_b.getOrigin().x(), a_to_b.getOrigin().y(), a_to_b.getOrigin().z());
	pso_data->tf_transformer[particle_id]->lookupTransform("/car", "/gps", tf::Time(0), a_to_b);
//	printf("gps with respect to car: x: %lf, y: %lf, z: %lf\n", a_to_b.getOrigin().x(), a_to_b.getOrigin().y(), a_to_b.getOrigin().z());
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
	double new_phi = phi / (1.0 + v * v * 0.00); // underster IARA
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
compute_phi_spline(double particle_a, double particle_b, double max_steering_angle, PsoData *pso_data, int particle_id)
{
//	double k1 = (1.0 / 3.0) * MAX_STEERING_ANGLE + particle_a;
//	double k2 = (2.0 / 3.0) * MAX_STEERING_ANGLE + particle_b;
//
//	double knots_x[NUM_PHI_SPLINE_KNOTS] = {0.0, (1.0 / 3.0) * MAX_STEERING_ANGLE, (2.0 / 3.0) * MAX_STEERING_ANGLE, MAX_STEERING_ANGLE};
//	double knots_y[NUM_PHI_SPLINE_KNOTS] = {0.0, k1, k2, MAX_STEERING_ANGLE};

	double k1 = (1.0 / 2.0 + particle_a) * max_steering_angle + particle_b;

	double knots_x[NUM_PHI_SPLINE_KNOTS] = {0.0, (1.0 / 2.0 + particle_a) * max_steering_angle, max_steering_angle};
	double knots_y[NUM_PHI_SPLINE_KNOTS] = {0.0, k1, max_steering_angle};

	gsl_spline_init(pso_data->phi_spline[particle_id], knots_x, knots_y, NUM_PHI_SPLINE_KNOTS);
}


void
compute_optimized_odometry_pose(double &x, double &y, double &yaw, double *particle,
	double optimized_v, double raw_phi, double dt, double L, gsl_spline *phi_spline)
{
	// phi = raw_phi + add_bias
	double phi = raw_phi + particle[3];

	if (use_non_linear_phi)
	{
		if (phi < 0.0)
			phi = -gsl_spline_eval(phi_spline, -phi, acc);
		else
			phi = gsl_spline_eval(phi_spline, phi, acc);
	}
	// phi = raw_phi * mult_bias
	phi = phi * particle[2];

	ackerman_model(x, y, yaw, optimized_v, phi, dt, L);
}


void
transform_car_to_gps(double car_x, double car_y, double *gps_x, double *gps_y, Transform &car2gps)
{
	Vector3 p_car;
	Vector3 p_gps;

	p_car = Vector3(car_x, car_y, 0);
	p_gps = car2gps * p_car;

	*gps_x = p_gps.getX();
	*gps_y = p_gps.getY();
}


void
get_gps_pose_given_odomentry_pose(double &gps_x, double &gps_y, double x, double y, double yaw, Transformer *tf_transformer)
{
	tf::Transform world_to_car;
	world_to_car.setOrigin(Vector3(x, y, 0.0));
	world_to_car.setRotation(Quaternion(yaw, 0.0, 0.0));
	tf_transformer->setTransform(tf::StampedTransform(world_to_car, tf::Time(0), "/world", "/car"));

	tf::StampedTransform world_to_gps;
	tf_transformer->lookupTransform("/world", "/gps", tf::Time(0), world_to_gps);

	gps_x = world_to_gps.getOrigin().x();
	gps_y = world_to_gps.getOrigin().y();
}


void
get_odomentry_pose_given_gps_pose(double &x, double &y, /* double gps_x, double gps_y, */ double yaw, Transformer *tf_transformer)
{
	tf::StampedTransform gps_to_car;
	tf_transformer->lookupTransform("/gps", "/car", tf::Time(0), gps_to_car);

	double x_ = gps_to_car.getOrigin().x();
	double y_ = gps_to_car.getOrigin().y();
	// transformacao da pose do carro do sistema de coordenadas do gps para o mundo assumindo que a translacao eh zero.
	x = x_ * cos(yaw) - y_ * sin(yaw);
	y = x_ * sin(yaw) + y_ * cos(yaw);
}


void
print_result(double *particle, FILE *f_report, PsoData *pso_data, int *id_first_pose, int particle_id)
{
	double yaw = particle[4];
	double x;
	double y;
	get_odomentry_pose_given_gps_pose(x, y, yaw, pso_data->tf_transformer[particle_id]); // gps comecca na pose zero

	double unoptimized_yaw = yaw;
	//yaw = yaw_withoutbias = estimate_theta(pso_data, 0);

	double unoptimized_x = 0.0;
	double unoptimized_y = 0.0;

	if (use_non_linear_phi)
		compute_phi_spline(particle[5], particle[6], pso_data->max_steering_angle, pso_data, particle_id);

	fprintf(f_report, "Initial angle: %lf\n", yaw);
//	fprintf(stderr, "Initial angle: %lf\n", yaw);

	double dt_gps_and_odom_acc = 0.0;
	int first_sample = -1;

	for (uint i = 1; i < pso_data->lines.size(); i++)
	{
		double v = pso_data->lines[i - 1].v * particle[0] + particle[1];

		pso_data->lines[i].opt_odom_valid = false;
		if (fabs(v) > MIN_VELOCITY)
		{
			double gps_x;
			double gps_y;
			double dt_gps_and_odom;
			double odometry_in_gps_coordinates_x, odometry_in_gps_coordinates_y;
			if (first_sample == -1)
			{
				first_sample = i - 1;
				gps_x = 0.0;
				gps_y = 0.0;

				get_gps_pose_given_odomentry_pose(odometry_in_gps_coordinates_x, odometry_in_gps_coordinates_y, x, y, yaw, pso_data->tf_transformer[particle_id]);
				pso_data->lines[first_sample].opt_odom_x = x;
				pso_data->lines[first_sample].opt_odom_y = y;
				pso_data->lines[first_sample].opt_odom_yaw = yaw;
				pso_data->lines[first_sample].opt_odom_valid = true;

				dt_gps_and_odom = fabs(pso_data->lines[first_sample].time - pso_data->lines[first_sample].gps_time);
				dt_gps_and_odom_acc += dt_gps_and_odom;

				fprintf(f_report, "DATA %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
						odometry_in_gps_coordinates_x, odometry_in_gps_coordinates_y, gps_x, gps_y, unoptimized_x, unoptimized_y, dt_gps_and_odom, dt_gps_and_odom_acc, x, y);
			}

			double dt = pso_data->lines[i].gps_time - pso_data->lines[i - 1].gps_time;
			compute_optimized_odometry_pose(x, y, yaw, particle, v, pso_data->lines[i].phi, dt, pso_data->distance_between_front_and_rear_axles, pso_data->phi_spline[particle_id]);
			pso_data->lines[i].opt_odom_x = x;
			pso_data->lines[i].opt_odom_y = y;
			pso_data->lines[i].opt_odom_yaw = yaw;
			pso_data->lines[i].opt_odom_valid = true;

			gps_x = pso_data->lines[i].gps_x - pso_data->lines[first_sample].gps_x;
			gps_y = pso_data->lines[i].gps_y - pso_data->lines[first_sample].gps_y;

			get_gps_pose_given_odomentry_pose(odometry_in_gps_coordinates_x, odometry_in_gps_coordinates_y, x, y, yaw, pso_data->tf_transformer[particle_id]);

			dt_gps_and_odom = fabs(pso_data->lines[i].time - pso_data->lines[i].gps_time);
			dt_gps_and_odom_acc += dt_gps_and_odom;

			ackerman_model(unoptimized_x, unoptimized_y, unoptimized_yaw, pso_data->lines[i].v, pso_data->lines[i].phi, dt,
										 pso_data->distance_between_front_and_rear_axles);

			fprintf(f_report, "DATA %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
					odometry_in_gps_coordinates_x, odometry_in_gps_coordinates_y, gps_x, gps_y, unoptimized_x, unoptimized_y, dt_gps_and_odom, dt_gps_and_odom_acc, x, y);
		}
	}

	*id_first_pose = first_sample;
}


double 
fitness(double *particle, void *data, int particle_id)
{
	PsoData *pso_data = (PsoData *) data;

	double yaw = particle[4];
	double x;
	double y;
	get_odomentry_pose_given_gps_pose(x, y, yaw, pso_data->tf_transformer[particle_id]); // gps comecca na pose zero
	
	if (use_non_linear_phi)
		compute_phi_spline(particle[5], particle[6], pso_data->max_steering_angle, pso_data, particle_id);

	double error = 0.0;
	double count = 0;

	int first_sample = -1;
	for (uint i = 1; i < pso_data->lines.size(); i++)
	{
		double v = pso_data->lines[i - 1].v * particle[0] + particle[1];
		if (fabs(v) > MIN_VELOCITY)
		{
			if (first_sample == -1)
				first_sample = i - 1;

			double dt = pso_data->lines[i].gps_time - pso_data->lines[i - 1].gps_time;
			compute_optimized_odometry_pose(x, y, yaw, particle, v, pso_data->lines[i].phi, dt, pso_data->distance_between_front_and_rear_axles, pso_data->phi_spline[particle_id]);

			// translate the starting pose of gps to zero to avoid floating point numerical instability
			double gps_x = pso_data->lines[i].gps_x - pso_data->lines[first_sample].gps_x;
			double gps_y = pso_data->lines[i].gps_y - pso_data->lines[first_sample].gps_y;

			double odometry_in_gps_coordinates_x, odometry_in_gps_coordinates_y;
			get_gps_pose_given_odomentry_pose(odometry_in_gps_coordinates_x, odometry_in_gps_coordinates_y, x, y, yaw, pso_data->tf_transformer[particle_id]);

			// add the error
			error += sqrt(pow(odometry_in_gps_coordinates_x - gps_x, 2.0) + pow(odometry_in_gps_coordinates_y - gps_y, 2.0));

			// reinforce consistency between heading direction and heading estimated using gps
//			double gps_yaw = atan2(pso_data->lines[i].gps_y - pso_data->lines[i - 1].gps_y,
//								   pso_data->lines[i].gps_x - pso_data->lines[i - 1].gps_x);
//
//			error += 200.0 * fabs(carmen_normalize_theta(gps_yaw - yaw));
			count += 1.0;
		}
	}

	// pso only maximizes, so we use as fitness the inverse of the error.
	return (-error / count);
}


size_t
find_nearest_time_to_gps(PsoData *pso_data, int id_gps)
{
	size_t i = 0;

	while (pso_data->velodyne_data[i].velodyne_timestamp < pso_data->lines[id_gps].gps_time)
		i++;

	return (i);
}


void
save_poses_in_graphslam_format(ParticleSwarmOptimization &optimizer, PsoData *pso_data, const string &path, int id_first_sample, int particle_id)
{
	if (path.size() <= 0)
		return;

	double *particle = optimizer.GetBestSolution();
	FILE *fptr = safe_fopen(path.c_str(), "w");

	double yaw;
	double x;
	double y;

	if (use_non_linear_phi)
		compute_phi_spline(particle[5], particle[6], pso_data->max_steering_angle, pso_data, particle_id);

	int first_sample = -1;
	size_t velodyne_sample;
	double timestamp;
	for (uint i = 1; i < pso_data->lines.size(); i++)
	{
		double v = pso_data->lines[i - 1].v * particle[0] + particle[1];

		if (fabs(v) > MIN_VELOCITY)
		{
			double global_x;
			double global_y;

			if (first_sample == -1)
			{
				first_sample = i - 1;
				velodyne_sample = find_nearest_time_to_gps(pso_data, id_first_sample);
			}

			if (!pso_data->lines[i - 1].opt_odom_valid)
				continue;

			x = pso_data->lines[i - 1].opt_odom_x;
			y = pso_data->lines[i - 1].opt_odom_y;
			yaw = pso_data->lines[i - 1].opt_odom_yaw;

			timestamp = pso_data->lines[i - 1].gps_time;
			while ((timestamp < pso_data->lines[i].gps_time) && (velodyne_sample < pso_data->velodyne_data.size()))
			{
				double dt = pso_data->velodyne_data[velodyne_sample].velodyne_timestamp - timestamp;
				compute_optimized_odometry_pose(x, y, yaw, particle, pso_data->velodyne_data[velodyne_sample].v, pso_data->velodyne_data[velodyne_sample].phi, dt,
					pso_data->distance_between_front_and_rear_axles, pso_data->phi_spline[particle_id]);
				global_x = x + pso_data->lines[first_sample].gps_x;
				global_y = y + pso_data->lines[first_sample].gps_y;

				fprintf(fptr, "%lf %lf %lf %lf\n", global_x, global_y, yaw,
								pso_data->velodyne_data[velodyne_sample].velodyne_timestamp);

				timestamp = pso_data->velodyne_data[velodyne_sample].velodyne_timestamp;
				velodyne_sample++;
			}
		}
	}

	fclose(fptr);
}


void
print_optimization_report(FILE *f_calibration, FILE *f_report, ParticleSwarmOptimization *optimizer)
{
	if (use_non_linear_phi)
	{
		if (f_calibration)
			fprintf(f_calibration,
					"v (multiplier bias): (%lf %lf),  phi (multiplier bias): (%lf %lf),  Initial Angle: %lf,  k1: %lf,  k2: %lf\n",
					optimizer->GetBestSolution()[0], optimizer->GetBestSolution()[1],
					optimizer->GetBestSolution()[2], optimizer->GetBestSolution()[3],
					optimizer->GetBestSolution()[4], optimizer->GetBestSolution()[5],
					optimizer->GetBestSolution()[6]);

		fprintf(stderr,
				"v (multiplier bias): (%lf %lf),  phi (multiplier bias): (%lf %lf),  Initial Angle: %lf,  k1: %lf,  k2: %lf\n",
				optimizer->GetBestSolution()[0], optimizer->GetBestSolution()[1],
				optimizer->GetBestSolution()[2], optimizer->GetBestSolution()[3],
				optimizer->GetBestSolution()[4], optimizer->GetBestSolution()[5],
				optimizer->GetBestSolution()[6]);
	}
	else
	{
		if (f_calibration)
			fprintf(f_calibration,
					"v (multiplier bias): (%lf %lf),  phi (multiplier bias): (%lf %lf),  Initial Angle: %lf\n",
					optimizer->GetBestSolution()[0], optimizer->GetBestSolution()[1],
					optimizer->GetBestSolution()[2], optimizer->GetBestSolution()[3],
					optimizer->GetBestSolution()[4]);

		fprintf(stderr,
				"v (multiplier bias): (%lf %lf),  phi (multiplier bias): (%lf %lf),  Initial Angle: %lf\n",
				optimizer->GetBestSolution()[0], optimizer->GetBestSolution()[1],
				optimizer->GetBestSolution()[2], optimizer->GetBestSolution()[3],
				optimizer->GetBestSolution()[4]);
	}

	if (f_report)
	{
		fprintf(f_report, "Fitness (MSE): %lf\n", optimizer->GetBestFitness());
		fprintf(f_report, "Fitness (SQRT(MSE)): %lf\n", sqrt(fabs(optimizer->GetBestFitness())));
		fprintf(stderr, "Fitness (MSE): %lf\n", optimizer->GetBestFitness());
		fprintf(stderr, "Fitness (SQRT(MSE)): %lf\n", sqrt(fabs(optimizer->GetBestFitness())));
	}
}


void
plot_graph(ParticleSwarmOptimization *optimizer, void *data, int particle_id)
{
	static bool first_time = true;
	double *particle;

	PsoData *pso_data = (PsoData *) data;

	if (!pso_data->view_active)
		return;

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

	int first_sample;
	FILE *gnuplot_data_file = fopen("gnuplot_data.txt", "w");
	print_result(particle, gnuplot_data_file, pso_data, &first_sample, particle_id);
	fclose(gnuplot_data_file);

	save_poses_in_graphslam_format(*optimizer, pso_data, "/tmp/graph_poses.txt", first_sample, particle_id);

	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_data.txt' u 10:11 w l lt rgb 'blue'  t 'odometry in car coordinates',"
			"'./gnuplot_data.txt' u 2:3 w l   lt rgb 'green' t 'odometry in gps coordinates',"
			"'./gnuplot_data.txt' u 4:5 w l   lt rgb 'red'   t 'gps',"
			"'/tmp/graph_poses.txt' u ($1 - %lf):($2 - %lf) w l lt rgb 'black' t 'odometry in graphslam format'\n",
			pso_data->lines[first_sample].gps_x, pso_data->lines[first_sample].gps_y);

	fflush(gnuplot_pipe);

	print_optimization_report(NULL, NULL, optimizer);
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


double **
set_limits(int dim, CommandLineArguments *args)
{
	double **limits;

	limits = alloc_limits(dim);

	// v multiplicative bias
	//limits[0][0] = 0.95; //0.5;
	//limits[0][1] = 1.05; //1.5;
	limits[0][0] = args->get<double>("min_multiplicative_v");
	limits[0][1] = args->get<double>("max_multiplicative_v");

	// v additive bias
	limits[1][0] = -0.00000001;
	limits[1][1] = 0.00000001;

	// phi multiplicative bias
	limits[2][0] = args->get<double>("min_multiplicative_phi");
	limits[2][1] = args->get<double>("max_multiplicative_phi");

	// phi additive bias
	limits[3][0] = carmen_degrees_to_radians(args->get<double>("min_additive_phi"));
	limits[3][1] = carmen_degrees_to_radians(args->get<double>("max_additive_phi"));

	// Initial angle
	limits[4][0] = -M_PI;
	limits[4][1] = M_PI;

	if (use_non_linear_phi)
	{
		// k1 of phi spline
		limits[5][0] = args->get<double>("min_k1");
		limits[5][1] = args->get<double>("max_k1");

		// k2 of phi spline
		limits[6][0] = args->get<double>("min_k2");
		limits[6][1] = args->get<double>("max_k2");
	}

	return (limits);
}


void
declare_and_parse_args(int argc, char **argv, CommandLineArguments *args)
{
	args->add_positional<string>("log_path", "Path to a log");
	args->add_positional<string>("carmen_ini", "Path to a file containing system parameters");
	args->add_positional<string>("output_calibration", "Path to an output calibration file");
	args->add_positional<string>("output_poses", "Path to a file in which poses will be saved for debug");
	args->add_positional<string>("poses_opt", "Path to a file in which the poses will be saved in graphslam format");
	args->add<int>("gps_to_use", "Id of the gps that will be used for the calibration", 1);
	args->add<int>("board_to_use", "Id of the sensor board that will be used for the calibration", 1);
	args->add<int>("use_non_linear_phi", "0 - linear phi; 1 - use a spline to map phi to a new phi", 0);
	args->add<int>("n_particles,n", "Number of particles", 500);
	args->add<int>("n_iterations,i", "Number of iterations", 300);
	args->add<int>("initial_log_line,l", "Number of lines to skip in the beggining of the log file", 1);
	args->add<int>("max_log_lines,m", "Maximum number of lines to read from the log file", -1);
	args->add<int>("view", "Flag indicating if the visualization should run or not.", 1);
	args->add<double>("min_multiplicative_v", "Lower limit of velocity multiplier", 0.979999);
	args->add<double>("max_multiplicative_v", "Upper limit of velocity multiplier", 1.3001);
	args->add<double>("min_multiplicative_phi", "Lower limit of phi multiplier", 0.55);
	args->add<double>("max_multiplicative_phi", "Upper limit of phi multiplier", 2.5);
	args->add<double>("min_additive_phi", "Lower limit of phi additive bias (degrees)", -2);
	args->add<double>("max_additive_phi", "Upper limit of phi additive bias (degrees)", 2);
	args->add<double>("min_k1", "Lower limit of k1 spline coefficient", -0.3);
	args->add<double>("max_k1", "Upper limit of k1 spline coefficient", 0.3);
	args->add<double>("min_k2", "Lower limit of k2 spline coefficient", -0.15);
	args->add<double>("max_k2", "Upper limit of k2 spline coefficient", 0.15);
	args->save_config_file("odom_calib_config.txt");
	args->parse(argc, argv);
}


void
initialize_parameters(PsoData &pso_data, CommandLineArguments *args, CarmenParamFile *carmen_ini_params)
{
	gps_to_use = args->get<int>("gps_to_use");
	board_to_use = args->get<int>("board_to_use");

	use_non_linear_phi = args->get<int>("use_non_linear_phi");

	if (use_non_linear_phi)
		n_params = 7;
	else
		n_params = 5;

	limits = set_limits(n_params, args);

	pso_data.max_steering_angle = carmen_ini_params->get<double>("robot_max_steering_angle");
	pso_data.distance_between_front_and_rear_axles = carmen_ini_params->get<double>("robot_distance_between_front_and_rear_axles");

	acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	int num_particles = args->get<int>("n_particles");
	pso_data.phi_spline = (gsl_spline **) malloc(num_particles * sizeof(gsl_spline *));
	pso_data.tf_transformer = (Transformer **) malloc(num_particles * sizeof(Transformer *));
	for (int i = 0; i < num_particles; i++)
	{
		pso_data.phi_spline[i] = gsl_spline_alloc(type, NUM_PHI_SPLINE_KNOTS);
		pso_data.tf_transformer[i] = new tf::Transformer(false);
		initialize_car_objects_poses_in_transformer(carmen_ini_params, &pso_data, gps_to_use, board_to_use, i);
	}
}


int 
main(int argc, char **argv)
{
	CommandLineArguments args;

	declare_and_parse_args(argc, argv, &args);
	CarmenParamFile *carmen_ini_params = new CarmenParamFile(args.get<string>("carmen_ini").c_str());
	PsoData pso_data;

	initialize_parameters(pso_data, &args, carmen_ini_params);

	read_data(args.get<string>("log_path").c_str(), gps_to_use,
						args.get<int>("initial_log_line"),
						args.get<int>("max_log_lines"),
						&pso_data);

	pso_data.view_active = args.get<int>("view");

	FILE *f_calibration = safe_fopen(args.get<string>("output_calibration").c_str(), "w");
	FILE *f_report = safe_fopen(args.get<string>("output_poses").c_str(), "w");

	srand(time(NULL));
	srand(rand()); // ??

//	double x = 0.0;
//	double y = 0.0;
//	double yaw = 0.0;
//	get_odomentry_pose_given_gps_pose(x, y, yaw, &pso_data); // gps comecca na pose zero
//	double odometry_in_gps_coordinates_x, odometry_in_gps_coordinates_y;
//	get_gps_pose_given_odomentry_pose(odometry_in_gps_coordinates_x, odometry_in_gps_coordinates_y, x, y, yaw, &pso_data);
//	printf("x = %lf, y = %lf, g_x = %lf, g_y = %lf\n", x, y, odometry_in_gps_coordinates_x, odometry_in_gps_coordinates_y);
//	getchar();

	ParticleSwarmOptimization optimizer(fitness, limits, n_params, &pso_data, args.get<int>("n_particles"), args.get<int>("n_iterations"));

	optimizer.Optimize(plot_graph);

	int first_sample;
	print_optimization_report(f_calibration, f_report, &optimizer);
	print_result(optimizer.GetBestSolution(), f_report, &pso_data, &first_sample, optimizer.GetBestParticleId());
	save_poses_in_graphslam_format(optimizer, &pso_data, args.get<string>("poses_opt"), first_sample, optimizer.GetBestParticleId());
	plot_graph(&optimizer, (void *) &pso_data, optimizer.GetBestParticleId());

	if (use_non_linear_phi)
		print_phi_spline(pso_data.phi_spline[optimizer.GetBestParticleId()], acc, pso_data.max_steering_angle, true);

	for (int i = 0; i < args.get<int>("n_particles"); i++)
	{
		gsl_spline_free(pso_data.phi_spline[i]);
		delete(pso_data.tf_transformer[i]);
	}
	free(pso_data.phi_spline);
	free(pso_data.tf_transformer);
	gsl_interp_accel_free(acc);

	delete(carmen_ini_params);

	if (args.get<int>("view"))
	{
		fprintf(stderr, "Press a key to finish...\n");
		getchar();
		fclose(gnuplot_pipe);
	}

	return (0);
}

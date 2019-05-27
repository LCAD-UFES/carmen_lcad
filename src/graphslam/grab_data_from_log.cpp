#include <vector>
#include <carmen/carmen.h>
#include <carmen/util_io.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/Gdc_Coord_3d.h>
#include <carmen/Utm_Coord_3d.h>
#include <carmen/Gdc_To_Utm_Converter.h>

#include "velodyne_util.h"

#define MAX_LINE_LENGTH (5 * 4000000)

using namespace std;

typedef struct
{
	carmen_orientation_3D_t orientation;
	double time;
}StampedOrientation;

typedef struct
{
	double v;
	double phi;
	double time;
}StampedOdometry;

typedef struct
{
	carmen_pose_3D_t pose;
	double time;
}StampedPose;


FILE *output_file;

vector<StampedOrientation> gps_orientation_queue;
vector<StampedOdometry> odometry_queue;
vector<StampedPose> gps_queue;
vector<double> gps_queue_stds;
vector<int> gps_orientation_valid_queue;

static const int GPS_QUEUE_MAX_SIZE = 100;
static const int ODOMETRY_QUEUE_MAX_SIZE = 100;

double distance_between_front_and_rear_axles;


template<class message_type> int
find_nearest_message(vector<message_type> &queue, double reference_sensor_time, const char *sensor_name)
{
	int i;
	double time_diff;
	static int num_warnings = 0;

	int last = queue.size() - 1;

	for (i = last; i >= 0; i--)
	{
		time_diff = reference_sensor_time - queue[i].time;

		if (time_diff > 0.0)
		{
			if (time_diff > 0.3)
			{
				printf("Warning %d: %s time_diff > 0.3\n", ++num_warnings, sensor_name);
				fflush(stdout);
			}

			return i;
		}
	}

	return (-1);
}


template<class T> void
add_message_to_queue(vector<T> &queue, T message, int max_size __attribute__((unused)))
{
	queue.push_back(message);
}


void
ackerman_prediction(carmen_pose_3D_t *pose, double v, double phi, double dt, double distance_between_front_and_rear_axles)
{
	pose->position.x = pose->position.x + dt * v * cos(pose->orientation.yaw);
	pose->position.y = pose->position.y + dt * v * sin(pose->orientation.yaw);
	pose->orientation.yaw = pose->orientation.yaw + dt * (v / distance_between_front_and_rear_axles) * tan(phi);
	pose->orientation.yaw = carmen_normalize_theta(pose->orientation.yaw);
}


void
write_sensor_data(carmen_pose_3D_t dead_reckoning, carmen_pose_3D_t gps_pose, double time, double gps_std, double gps_yaw, int gps_valid)
{
	fprintf(output_file, "%lf %lf %lf %lf %lf %lf 0.0 0.0 0.0 %lf %lf %lf %d\n",
		dead_reckoning.position.x, dead_reckoning.position.y, dead_reckoning.orientation.yaw,
		gps_pose.position.x, gps_pose.position.y, gps_pose.orientation.yaw,
		time, gps_std, gps_yaw, gps_valid);

	fflush(output_file);
}


void
velodyne_handler(double velodyne_timestamp, double initial_timestamp, double final_timestamp,
		double distance_between_front_and_rear_axles, double initial_odometry_angle)
{
	double dt;
	double dt_gps;
	double gps_orientation;
	carmen_pose_3D_t gps_pose;
	int gps_orientation_valid;
	static double last_time;
	static bool is_first = true;
	static carmen_pose_3D_t dead_reckoning = {{0.0, 0.0, 0.0}, {0.0, 0.0, initial_odometry_angle}};

	int gpsid = find_nearest_message<StampedPose>(gps_queue, velodyne_timestamp, "gps to velodyne");
	int odomid = find_nearest_message<StampedOdometry>(odometry_queue, velodyne_timestamp, "odometry to velodyne");

	if ((velodyne_timestamp >= initial_timestamp) && (velodyne_timestamp <= final_timestamp) &&
		(gpsid >= 0) && (odomid >= 0))
	{
		if (!is_first) // ignore the first message to get a valid last_time
		{
			dt = velodyne_timestamp - last_time;
			dt_gps = velodyne_timestamp - gps_queue[gpsid].time;

			gps_pose = gps_queue[gpsid].pose;
			gps_orientation = gps_pose.orientation.yaw;
			gps_orientation_valid = 1;

			double gps_std = gps_queue_stds[gpsid];

			if ((dt <= 0) || (dt_gps <= 0))
				printf("** ERROR: (dt <= 0) || (dt_gps <= 0) in velodyne_handler()\n");

			ackerman_prediction(&dead_reckoning, odometry_queue[odomid].v, odometry_queue[odomid].phi, dt, distance_between_front_and_rear_axles);
			ackerman_prediction(&gps_pose, odometry_queue[odomid].v, odometry_queue[odomid].phi, dt_gps, distance_between_front_and_rear_axles);

			write_sensor_data(dead_reckoning, gps_pose, velodyne_timestamp, gps_std, gps_orientation, gps_orientation_valid);
		}
		else
			is_first = false;

		last_time = velodyne_timestamp;
	}
}


void
gps_xyz_message_handler(carmen_gps_xyz_message *message, int gps_to_use, double gps_latency)
{
	if (message->nr != gps_to_use)
		return;

	StampedPose pose;
	pose.pose.position.x = message->x;
	pose.pose.position.y = message->y;
	pose.pose.position.z = 0.0;
	pose.pose.orientation.roll = 0.0;
	pose.pose.orientation.pitch = 0.0;
	if (gps_queue.size() > 0)
		pose.pose.orientation.yaw = atan2(message->y - gps_queue[gps_queue.size() - 1].pose.position.y, message->x - gps_queue[gps_queue.size() - 1].pose.position.x);
	else
		pose.pose.orientation.yaw = 0.0; // A primeira mensagem nao é usada em velodyne_handler()
	pose.time = message->timestamp - gps_latency;

	double gps_std;

	// @Filipe: desvios padrao para cada modo do GPS Trimble.
	// @Filipe: OBS: Multipliquei os stds por 2 no switch abaixo para dar uma folga.
	// 0: DBL_MAX
	// 1: 4.0
	// 2: 1.0
	// 4: 0.1
	// 5: 0.1

	switch (message->gps_quality)
	{
		case 1:
			gps_std = 8.0;
			break;
		case 2:
			gps_std = 2.0;
			break;
		case 4:
			gps_std = 0.2;
			break;
		case 5:
			gps_std = 1.5;
			break;
		default:
			gps_std = DBL_MAX;
	}

	add_message_to_queue<double>(gps_queue_stds, gps_std, GPS_QUEUE_MAX_SIZE);
	add_message_to_queue<StampedPose>(gps_queue, pose, GPS_QUEUE_MAX_SIZE);
}


void
robot_ackerman_handler(carmen_robot_ackerman_velocity_message *odometry,
		double v_multiplier, double phi_multiplier, double phi_bias)
{
	StampedOdometry odom;

	odom.v = odometry->v * v_multiplier;
	odom.phi = odometry->phi * phi_multiplier + phi_bias;
	odom.time = odometry->timestamp;

	add_message_to_queue<StampedOdometry>(odometry_queue, odom, ODOMETRY_QUEUE_MAX_SIZE);
}


double
read_velodyne_timestamp(FILE *f)
{
	double velodyne_timestamp;
	char scan_file_name[2048];
	int num_shots;

	fscanf(f, "%s %d %lf", scan_file_name, &num_shots, &velodyne_timestamp);

	return (velodyne_timestamp);
}


carmen_robot_ackerman_velocity_message
read_odometry(FILE *f)
{
	carmen_robot_ackerman_velocity_message m;
	fscanf(f, "%lf %lf %lf", &m.v, &m.phi, &m.timestamp);
	return (m);
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
	m.nr = gps_id;

	if (gps_to_use == gps_id)
	{
		fscanf(f, "%s", dummy);

		fscanf(f, "%lf", &lt_dm);
		fscanf(f, " %c ", &lt_orientation); // read a char ignoring space
		fscanf(f, "%lf", &lg_dm);
		fscanf(f, " %c ", &lg_orientation); // read a char ignoring space
		fscanf(f, "%d", &quality);
		m.gps_quality = quality;

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

	return (m);
}


void
generate_sync_file(const char *filename, int gps_to_use, double gps_latency, double initial_time, double final_time,
		double v_multiplier, double phi_multiplier, double phi_bias, double distance_between_front_and_rear_axles,
		double initial_odometry_angle)
{
	static char tag[256];
	static char line[MAX_LINE_LENGTH];

	vector<carmen_robot_ackerman_velocity_message> odoms;

	printf("Reading log...\n");
	FILE *f = safe_fopen(filename, "r");

	int num_gps_messages = 0;
	double first_velodyne_timestamp = 0.0;
	while(!feof(f))
	{
		fscanf(f, "\n%s", tag);

		if (!strcmp(tag, "NMEAGGA"))
		{
			carmen_gps_xyz_message m = read_gps(f, gps_to_use);
			gps_xyz_message_handler(&m, gps_to_use, gps_latency);
			num_gps_messages++;
		}
		else if (!strcmp(tag, "ROBOTVELOCITY_ACK"))
		{
			carmen_robot_ackerman_velocity_message m = read_odometry(f);
			robot_ackerman_handler(&m, v_multiplier, phi_multiplier, phi_bias);
		}
		else if (!strcmp(tag, "VELODYNE_PARTIAL_SCAN_IN_FILE"))
		{
			double velodyne_timestamp = read_velodyne_timestamp(f);
			if (first_velodyne_timestamp == 0.0)
				first_velodyne_timestamp = velodyne_timestamp;
			velodyne_handler(velodyne_timestamp, first_velodyne_timestamp + initial_time, first_velodyne_timestamp + final_time,
					distance_between_front_and_rear_axles, initial_odometry_angle); // This function generates the sync file.
		}

		fscanf(f, "%[^\n]\n", line);
	}

	fclose(f);
	printf("Done.\n");

	if (num_gps_messages == 0)
		exit(printf("Error: Unable to load data from log '%s'. Are you sure you logged gps '%d'?\n", filename, gps_to_use));
}


int
main(int argc, char **argv)
{
	if (argc < 4)
		exit(printf("Number of parameters invalid!\n Usage %s <log_file> <odometry_calibration_file> <sync_file> [<initial_time> <final_time>]\n", argv[0]));

	double initial_time = 0.0;
	double final_time = 9999999999999.0;
	if (argc == 5)
		initial_time = atof(argv[4]);
	if (argc == 6)
	{
		initial_time = atof(argv[4]);
		final_time = atof(argv[5]);
	}

	FILE *odometry_calibration_file = safe_fopen(argv[2], "r");
	double v_multiplier, v_bias, phi_multiplier, phi_bias, initial_angle, gps_latency, L;
	int gps_to_use;
	fscanf(odometry_calibration_file, "v (multiplier bias): (%lf %lf),  phi (multiplier bias): (%lf %lf),  Initial Angle: %lf, GPS to use: %d, GPS Latency: %lf, L: %lf",
			&v_multiplier, &v_bias, &phi_multiplier, &phi_bias, &initial_angle, &gps_to_use, &gps_latency, &L);
//	printf("v (multiplier bias): (%lf %lf),  phi (multiplier bias): (%lf %lf),  Initial Angle: %lf, GPS to use: %d, GPS Latency: %lf, L: %lf\n",
//			v_multiplier, v_bias, phi_multiplier, phi_bias, initial_angle, gps_to_use, gps_latency, L);

	output_file = safe_fopen(argv[3], "w");

	generate_sync_file(argv[1], gps_to_use, gps_latency, initial_time, final_time,
			v_multiplier, phi_multiplier, phi_bias, L, initial_angle);

	fclose(output_file);

	printf("Programa concluído normalmente. Tecle Ctrl+C para terminar\n");
	fflush(stdout);
	getchar();

	return (0);
}

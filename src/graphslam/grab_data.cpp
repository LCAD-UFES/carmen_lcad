#include <tf.h>
#include <vector>
#include <carmen/carmen.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/xsens_interface.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/fused_odometry_interface.h>
#include "velodyne_util.h"

using namespace std;
using namespace tf;

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


char *velodyne_path;
char *output_filename;
FILE *output_file;
vector<StampedOrientation> xsens_queue;
vector<StampedOrientation> gps_orientation_queue;
vector<StampedOdometry> odometry_queue;
vector<StampedPose> gps_queue;
vector<double> gps_queue_stds;
vector<int> gps_orientation_valid_queue;

static const int GPS_QUEUE_MAX_SIZE = 100;
static const int XSENS_QUEUE_MAX_SIZE = 100;
static const int ODOMETRY_QUEUE_MAX_SIZE = 100;

extern bool use_fused_odometry;
extern int gps_to_use;
extern double distance_between_front_and_rear_axles;


double
distance(double x1, double y1, double x2, double y2)
{
	double dx, dy, dist;

	dx = pow(x1 - x2, 2);
	dy = pow(y1 - y2, 2);
	dist = sqrt(dx + dy);

	return dist;
}


void
shutdown_grab_data(int sign)
{
	if (sign == SIGINT)
	{
		fclose(output_file);
		exit(0);
	}
}

/*
int
find_nearest_xsens(double time)
{
	uint i;
	int id;
	double time_diff;
	double min_time_diff = DBL_MAX;

	id = -1;

	for (i = 0; i < xsens_queue.size(); i++)
	{
		time_diff = fabs(time - xsens_queue[i].time);

		if (time_diff < min_time_diff)
		{
			min_time_diff = time_diff;
			id = i;
		}
	}

	return id;
}


int
find_nearest_gps_orientation(double time)
{
	uint i;
	int id;
	double time_diff;
	double min_time_diff = DBL_MAX;

	id = -1;

	for (i = 0; i < gps_orientation_queue.size(); i++)
	{
		time_diff = fabs(time - gps_orientation_queue[i].time);

		if (time_diff < min_time_diff)
		{
			min_time_diff = time_diff;
			id = i;
		}
	}

	return id;
}


int
find_nearest_odometry(double time)
{
	uint i;
	int id;
	double time_diff;
	double min_time_diff = DBL_MAX;

	id = -1;

	for (i = 0; i < odometry_queue.size(); i++)
	{
		time_diff = fabs(time - odometry_queue[i].time);

		if (time_diff < min_time_diff)
		{
			min_time_diff = time_diff;
			id = i;
		}
	}

	return id;
}


int
find_nearest_gps(double time)
{
	uint i;
	int id;
	double time_diff;
	double min_time_diff = DBL_MAX;

	id = -1;

	for (i = 0; i < gps_queue.size(); i++)
	{
		time_diff = gps_queue[i].time - time;

		if ((fabs(time_diff) < min_time_diff) && (time_diff > 0.0))
		{
			min_time_diff = time_diff;
			id = i;
		}
	}

	return id;
}
*/

template<class message_type> int
find_nearest_message(vector<message_type> &queue, double reference_sensor_time, const char *sensor_name)
{
	int i;
	double time_diff;

	int last = queue.size() - 1;

	for (i = last; i >= 0; i--)
	{
		time_diff = reference_sensor_time - queue[i].time;

		if (time_diff > 0.0)
		{
			if (time_diff > 0.2)
				printf("Warning: %s time_diff > 0.2\n", sensor_name);

			return i;
		}
	}

	return (-1);
}


void
measure_time_to_accumulate_cloud(carmen_velodyne_partial_scan_message *velodyne_message)
{
	static double n = 0.0;
	static double acc = 0.0;

	double elapsedTime;
	timeval t1, t2;
	gettimeofday(&t1, NULL);

	accumulate_clouds(velodyne_message, velodyne_path, 0, 0);

	gettimeofday(&t2, NULL);
	elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
	elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms

	n += 1.0;
	acc += elapsedTime;

	printf("time to accumulate cloud: %lf mean: %lf\n", elapsedTime, acc / n);
}


template<class T> void
add_message_to_queue(vector<T> &queue, T message, int max_size __attribute__((unused)))
{
	queue.push_back(message);
}


void
ackerman_prediction(carmen_pose_3D_t *pose, double v, double phi, double dt)
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
		time, gps_std, gps_yaw, gps_valid
	);

	fflush(output_file);
}


void
velodyne_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	//** DEBUG: the function below is used to measure the time spent during pointcloud processing.
	//** If its time is bigger than the velodyne frequency (= 1 / 20 = 0.05) you will lose information!!
	// measure_time_to_accumulate_cloud(velodyne_message);

	double dt;
	double dt_gps;
	double gps_orientation;
	carmen_pose_3D_t gps_pose;

	int gps_orientation_valid;
	static double last_time;
	static int is_first = 1;
	static carmen_pose_3D_t dead_reckoning = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

	//printf("*** GPS Queue size: %ld Odom Queue size: %ld\n", gps_queue.size(), odometry_queue.size());

	//int gpsid = find_nearest_gps(velodyne_message->timestamp);
	//int odomid = find_nearest_odometry(velodyne_message->timestamp);
	//int orid = find_nearest_gps_orientation(velodyne_message->timestamp);
	int gpsid = find_nearest_message<StampedPose>(gps_queue, velodyne_message->timestamp, "gps to velodyne");
	int odomid = find_nearest_message<StampedOdometry>(odometry_queue, velodyne_message->timestamp, "odometry to velodyne");
	//int orid = find_nearest_message<StampedOrientation>(gps_orientation_queue, velodyne_message->timestamp, "gps orientation to velodyne");

	if ((gpsid >= 0) && (odomid >= 0)) // && (orid >= 0))
	{
		if (!is_first) // ignore the first message to get a valid last_time
		{
			dt = velodyne_message->timestamp - last_time;
			dt_gps = velodyne_message->timestamp - gps_queue[gpsid].time;

			gps_pose = gps_queue[gpsid].pose;
			// gps_orientation = gps_orientation_queue[orid].orientation.yaw;
			// gps_orientation_valid = gps_orientation_valid_queue[orid];
			gps_orientation = gps_pose.orientation.yaw;
			gps_orientation_valid = 1;

			double gps_std = gps_queue_stds[gpsid];

			if ((dt <= 0) || (dt_gps <= 0))// || (odometry_queue[odomid].v < 0.05))
				printf("** ERROR: UNPREDICTED PROBLEM\n");

			ackerman_prediction(&dead_reckoning, odometry_queue[odomid].v, odometry_queue[odomid].phi, dt);
			ackerman_prediction(&gps_pose, odometry_queue[odomid].v, odometry_queue[odomid].phi, dt_gps);

			int saved_ok = accumulate_clouds(velodyne_message, velodyne_path, odometry_queue[odomid].v, odometry_queue[odomid].phi);

			if (saved_ok)
				//write_sensor_data(dead_reckoning, gps_pose, velodyne_message->timestamp, gps_std, gps_pose.orientation.yaw);
				write_sensor_data(dead_reckoning, gps_pose, velodyne_message->timestamp, gps_std, gps_orientation, gps_orientation_valid);
		}
		else
			is_first = 0;

		last_time = velodyne_message->timestamp;
	}
	else
		printf("*** ERROR: Empty GPS or Odometry Queue!!\n");
}


void
gps_xyz_message_handler(carmen_gps_xyz_message *message)
{
	if (use_fused_odometry)
		return;

	int xsid;
	double yaw, pitch, roll;

	if (message->nr != gps_to_use)
		return;

	//xsid = find_nearest_xsens(message->timestamp);
	xsid = find_nearest_message<StampedOrientation>(xsens_queue, message->timestamp, "xsens to gps");

	if (xsid < 0)
		return;
	else
	{
		// TODO: Medir o tempo gasto para criar o multiplicar as matrizes com TF. Se for lento, usar outra lib!
		Transform gps;
		Transform rotgps;
		Transform rotation;

		gps.setOrigin(Vector3(message->x, message->y, 0.0));
		gps.setRotation(Quaternion(0, 0, 0, 1));

		rotation.setOrigin(Vector3(0, 0, 0));
		rotation.setRotation(Quaternion(xsens_queue[xsid].orientation.yaw, 0.0, 0.0));

		//** Question: por que isso só funciona se a transformada for pela direita?
		//** O certo seria pela esquerda, mas ao fazer isso o resultado de qualquer percurso eh um circulo...
		rotgps = gps * rotation;
		xsens_queue.clear();

		Matrix3x3(rotgps.getRotation()).getEulerYPR(yaw, pitch, roll);

		StampedPose pose;

		// pose.pose.position.x = rotgps.getOrigin().x();
		// pose.pose.position.y = rotgps.getOrigin().y();
		// pose.pose.position.z = rotgps.getOrigin().z();
		// pose.pose.orientation.roll = roll;
		// pose.pose.orientation.pitch = pitch;
		// pose.pose.orientation.yaw = yaw;

		pose.pose.position.x = message->x;
		pose.pose.position.y = message->y;
		pose.pose.position.z = 0.0;
		pose.pose.orientation.roll = 0.0;
		pose.pose.orientation.pitch = 0.0;
		if (gps_queue.size() > 0)
			pose.pose.orientation.yaw = atan2(message->y - gps_queue[gps_queue.size() - 1].pose.position.y, message->x - gps_queue[gps_queue.size() - 1].pose.position.x);
		else
			pose.pose.orientation.yaw = 0.0;
		pose.time = message->timestamp;

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
}


void
base_ackermann_handler(carmen_base_ackerman_odometry_message *odometry)
{
	StampedOdometry odom;

	odom.v = odometry->v;
	odom.phi = odometry->phi;
	odom.time = odometry->timestamp;

	add_message_to_queue<StampedOdometry>(odometry_queue, odom, ODOMETRY_QUEUE_MAX_SIZE);
}


void
xsens_mtig_message_handler(carmen_xsens_mtig_message* xsens_mtig)
{
	if (use_fused_odometry)
		return;

	StampedOrientation data;
	Quaternion q(xsens_mtig->quat.q1, xsens_mtig->quat.q2, xsens_mtig->quat.q3, xsens_mtig->quat.q0);
	Matrix3x3(q).getEulerYPR(data.orientation.yaw, data.orientation.pitch, data.orientation.roll);
	data.time = xsens_mtig->timestamp;

	add_message_to_queue<StampedOrientation>(xsens_queue, data, XSENS_QUEUE_MAX_SIZE);
}


void
xsens_mti_message_handler(carmen_xsens_global_quat_message *xsens_mti)
{
	if (use_fused_odometry)
		return;

	StampedOrientation data;
	double *quat = xsens_mti->quat_data.m_data;
	Transform new_rotation;
	Quaternion q(quat[1], quat[2], quat[3], quat[0]);
	Matrix3x3(q).getEulerYPR(data.orientation.yaw, data.orientation.pitch, data.orientation.roll);
	data.time = xsens_mti->timestamp;

	add_message_to_queue<StampedOrientation>(xsens_queue, data, XSENS_QUEUE_MAX_SIZE);
}


void
gps_orientation_handler(carmen_gps_gphdt_message *message)
{
	if (use_fused_odometry)
		return;

	StampedOrientation data;

	data.orientation.yaw = message->heading;
	data.orientation.pitch = 0;
	data.orientation.roll = 0;
	data.time = message->timestamp;

	add_message_to_queue<StampedOrientation>(gps_orientation_queue, data, GPS_QUEUE_MAX_SIZE);
	add_message_to_queue<int>(gps_orientation_valid_queue, message->valid, GPS_QUEUE_MAX_SIZE);
}


double
get_particles_std(carmen_fused_odometry_particle_message *msg)
{
	double sum, mean, std_x = 0.0, std_y = 0.0;

	sum = 0.0;
	for (int i = 0; i < msg->num_particles; ++i)
		sum += msg->particle_pos[i].x;
	mean = sum / msg->num_particles;

	for (int i = 0; i < msg->num_particles; ++i)
		std_x += pow(msg->particle_pos[i].x - mean, 2.0);
	std_x = sqrt(std_x / msg->num_particles);

	sum = 0.0;
	for (int i = 0; i < msg->num_particles; ++i)
		sum += msg->particle_pos[i].y;
	mean = sum / msg->num_particles;

	for (int i = 0; i < msg->num_particles; ++i)
		std_y += pow(msg->particle_pos[i].y - mean, 2.0);
	std_y = sqrt(std_y / msg->num_particles);

	return ((std_x + std_y) / 2.0);
}


static void
fused_odometry_particle_message_handler(carmen_fused_odometry_particle_message *msg)
{
	if (!use_fused_odometry)
		return;

	StampedPose pose;

	pose.pose.position.x = msg->pose.position.x;
	pose.pose.position.y = msg->pose.position.y;
	pose.pose.position.z = msg->pose.position.z;

	pose.pose.orientation.roll = msg->pose.orientation.roll;
	pose.pose.orientation.pitch = msg->pose.orientation.pitch;
	pose.pose.orientation.yaw = msg->pose.orientation.yaw;

	pose.time = msg->timestamp;

	add_message_to_queue<StampedPose>(gps_queue, pose, GPS_QUEUE_MAX_SIZE);

	double gps_std = get_particles_std(msg);
	add_message_to_queue<double>(gps_queue_stds, gps_std, GPS_QUEUE_MAX_SIZE);

	StampedOrientation data;

	data.orientation.yaw = msg->pose.orientation.yaw;
	data.orientation.pitch = msg->pose.orientation.pitch;
	data.orientation.roll = msg->pose.orientation.roll;
	data.time = msg->timestamp;

	add_message_to_queue<StampedOrientation>(gps_orientation_queue, data, GPS_QUEUE_MAX_SIZE);
	add_message_to_queue<int>(gps_orientation_valid_queue, (gps_std < 0.5)? 1 : 0, GPS_QUEUE_MAX_SIZE);
}


void
subscribe_to_ipc_messages()
{
	carmen_base_ackerman_subscribe_odometry_message(NULL,
		(carmen_handler_t) base_ackermann_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_gps_xyz_subscribe_message(NULL,
		(carmen_handler_t) gps_xyz_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_xsens_subscribe_xsens_global_quat_message(NULL,
		(carmen_handler_t) xsens_mti_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_xsens_mtig_subscribe_message(NULL,
		(carmen_handler_t) xsens_mtig_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_gps_subscribe_nmea_hdt_message(NULL,
		  (carmen_handler_t) gps_orientation_handler,
		  CARMEN_SUBSCRIBE_LATEST);

	carmen_velodyne_subscribe_partial_scan_message(NULL,
		(carmen_handler_t) velodyne_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_fused_odometry_subscribe_fused_odometry_particle_message(NULL,
		(carmen_handler_t) fused_odometry_particle_message_handler,
		CARMEN_SUBSCRIBE_LATEST);
}


void
initialize_global_variables(char **argv)
{
	velodyne_path = argv[1];
	output_filename = argv[2];

	output_file = fopen(output_filename, "w");

	if (output_file == NULL)
		exit(printf("Unable to open file '%s'", output_filename));
}


int
main(int argc, char **argv)
{
	if (argc < 3)
	{
		printf("Use %s <velodyne-path> <output-filename>\n", argv[0]);
		exit(0);
	}

	initialize_global_variables(argv);
	carmen_randomize(&argc, &argv);
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_grab_data);
	load_parameters(argc, argv);

	subscribe_to_ipc_messages();
	carmen_ipc_dispatch();

	return 0;
}


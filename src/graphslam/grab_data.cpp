#include <tf.h>
#include <vector>
#include <carmen/carmen.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/xsens_interface.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/velodyne_interface.h>
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
vector<StampedOdometry> odometry_queue;
vector<StampedPose> gps_queue;


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
find_nearest_fused_gps(double time)
{
	uint i;
	int id;
	double time_diff;
	double min_time_diff = DBL_MAX;

	id = -1;

	for (i = 0; i < gps_queue.size(); i++)
	{
		time_diff = fabs(time - gps_queue[i].time);

		if (time_diff < min_time_diff)
		{
			min_time_diff = time_diff;
			id = i;
		}
	}

	return id;
}


void
measure_time_to_accumulate_cloud(carmen_velodyne_partial_scan_message *velodyne_message)
{
	static double n = 0.0;
	static double acc = 0.0;

	double elapsedTime;
	timeval t1, t2;
	gettimeofday(&t1, NULL);

	accumulate_clouds(velodyne_message, velodyne_path);

	gettimeofday(&t2, NULL);
	elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
	elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms

	n += 1.0;
	acc += elapsedTime;

	printf("time to accumulate cloud: %lf mean: %lf\n", elapsedTime, acc / n);
}


void
ackerman_prediction(carmen_pose_3D_t *pose, double v, double phi, double dt)
{
	pose->position.x = pose->position.x + dt * v * cos(pose->orientation.yaw);
	pose->position.y = pose->position.y + dt * v * sin(pose->orientation.yaw);
	pose->orientation.yaw = pose->orientation.yaw + dt * (v / 2.61874 /* L */) * tan(phi);
	pose->orientation.yaw = carmen_normalize_theta(pose->orientation.yaw);
}


void
show_data(carmen_pose_3D_t dead_reckoning, carmen_pose_3D_t gps_pose, double time)
{
	fprintf(output_file, "%lf %lf %lf %lf %lf %lf 0.0 0.0 0.0 %f\n",
		dead_reckoning.position.x, dead_reckoning.position.y, dead_reckoning.orientation.yaw,
		gps_pose.position.x, gps_pose.position.y, gps_pose.orientation.yaw,
		time
	);

	fflush(output_file);
}


void
velodyne_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	printf("VELODYNE\n");
	//** DEBUG: the function below is used to measure the time spent during pointcloud processing.
	//** If its time is bigger than the velodyne frequency (= 1 / 20 = 0.05) you will lose information!!
	// measure_time_to_accumulate_cloud(velodyne_message);

	double dt;
	double dt_gps;
	carmen_pose_3D_t gps_pose;

	static double last_time;
	static int is_first = 1;
	static carmen_pose_3D_t dead_reckoning = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

	int gpsid = find_nearest_fused_gps(velodyne_message->timestamp);
	int odomid = find_nearest_odometry(velodyne_message->timestamp);

	if ((gpsid >= 0) && (odomid >= 0))
	{
		if (!is_first) // ignore the first message to get a valid last_time
		{
			dt = fabs(velodyne_message->timestamp - last_time);
			dt_gps = fabs(velodyne_message->timestamp - gps_queue[gpsid].time);
			gps_pose = gps_queue[gpsid].pose;

			if ((dt <= 0) || (dt_gps <= 0) || (odometry_queue[odomid].v < 0.1))
				return;

			ackerman_prediction(&dead_reckoning, odometry_queue[odomid].v, odometry_queue[odomid].phi, dt);
			ackerman_prediction(&gps_pose, odometry_queue[odomid].v, odometry_queue[odomid].phi, dt_gps);
			show_data(dead_reckoning, gps_pose, velodyne_message->timestamp);

			accumulate_clouds(velodyne_message, velodyne_path);

			//printf("writing...\n");

			gps_queue.clear();
			odometry_queue.clear();
		}
		else
			is_first = 0;

		last_time = velodyne_message->timestamp;
	}
}


void
gps_xyz_message_handler(carmen_gps_xyz_message *message)
{
	printf("GPS\n");

	int xsid;
	double yaw, pitch, roll;

	xsid = find_nearest_xsens(message->timestamp);

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

		pose.pose.position.x = rotgps.getOrigin().x();
		pose.pose.position.y = rotgps.getOrigin().y();
		pose.pose.position.z = rotgps.getOrigin().z();

		pose.pose.orientation.roll = roll;
		pose.pose.orientation.pitch = pitch;
		pose.pose.orientation.yaw = yaw;

		pose.time = message->timestamp;

		gps_queue.push_back(pose);
	}
}


void
base_ackermann_handler(carmen_base_ackerman_odometry_message *odometry)
{
	printf("BASE\n");

	StampedOdometry odom;

	odom.v = odometry->v;
	odom.phi = odometry->phi;
	odom.time = odometry->timestamp;

	odometry_queue.push_back(odom);
}

void
xsens_mtig_message_handler(carmen_xsens_mtig_message* xsens_mtig)
{
	printf("XSENS\n");

	StampedOrientation data;
	Quaternion q(xsens_mtig->quat.q1, xsens_mtig->quat.q2, xsens_mtig->quat.q3, xsens_mtig->quat.q0);
	Matrix3x3(q).getEulerYPR(data.orientation.yaw, data.orientation.pitch, data.orientation.roll);
	data.time = xsens_mtig->timestamp;
	xsens_queue.push_back(data);
}

void
xsens_mti_message_handler(carmen_xsens_global_quat_message *xsens_mti)
{
	printf("XSENS\n");

	StampedOrientation data;
	double *quat = xsens_mti->quat_data.m_data;
	Transform new_rotation;
	Quaternion q(quat[1], quat[2], quat[3], quat[0]);
	Matrix3x3(q).getEulerYPR(data.orientation.yaw, data.orientation.pitch, data.orientation.roll);
	data.time = xsens_mti->timestamp;
	xsens_queue.push_back(data);
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
		(carmen_handler_t) xsens_mtig_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_velodyne_subscribe_partial_scan_message(NULL,
		(carmen_handler_t) velodyne_handler,
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


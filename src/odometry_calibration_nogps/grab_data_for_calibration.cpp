#include <tf.h>
#include <vector>
#include <carmen/carmen.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/xsens_interface.h>
#include <carmen/robot_ackerman_interface.h>

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


vector<StampedOrientation> xsens_queue;
vector<StampedOdometry> raw_command;

char *output_filename;
FILE *output_file;

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
shutdown_grab_data_for_calibration(int sign)
{
	if (sign == SIGINT)
	{
		fclose(output_file);
		exit(1);
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
find_nearest_raw_command(double time)
{
	uint i;
	int id;
	double time_diff;
	double min_time_diff = DBL_MAX;

	id = -1;

	for (i = 0; i < raw_command.size(); i++)
	{
		time_diff = fabs(time - raw_command[i].time);

		if (time_diff < min_time_diff)
		{
			min_time_diff = time_diff;
			id = i;
		}
	}

	return id;
}


void
ackerman_prediction(carmen_pose_3D_t *pose, double v, double phi, double dt)
{
	pose->position.x = pose->position.x + dt * v * cos(pose->orientation.yaw);
	pose->position.y = pose->position.y + dt * v * sin(pose->orientation.yaw);
	pose->orientation.yaw = pose->orientation.yaw + dt * (v / 2.625 /* L */) * tan(phi);
	pose->orientation.yaw = carmen_normalize_theta(pose->orientation.yaw);
}


int
get_rotated_gps(carmen_gps_xyz_message *message, StampedPose &pose)
{
	int xsid;
	double yaw, pitch, roll;

	xsid = find_nearest_xsens(message->timestamp);

	if (xsid < 0)
		return 0;

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

	pose.pose.position.x = rotgps.getOrigin().x();
	pose.pose.position.y = rotgps.getOrigin().y();
	pose.pose.position.z = rotgps.getOrigin().z();

	pose.pose.orientation.roll = roll;
	pose.pose.orientation.pitch = pitch;
	pose.pose.orientation.yaw = yaw;

	pose.time = message->timestamp;

	return 1;
}



void
gps_xyz_message_handler(carmen_gps_xyz_message *message)
{
	StampedPose pose;

	if (message->nr != 1) // GPS Trimble
		return;

	// it fuses gps and xsens to get gps with orientation
	if (!get_rotated_gps(message, pose))
		return;

	int rawid = find_nearest_raw_command(message->timestamp);

	if (rawid >= 0)
	{
		fprintf(output_file, "RAW %lf %lf %lf %lf %lf %lf %lf\n",
			raw_command[rawid].v, raw_command[rawid].phi, raw_command[rawid].time,
			pose.pose.position.x, pose.pose.position.y,
			pose.pose.orientation.yaw, pose.time
		);
		fflush(output_file);

		raw_command.clear();
	}
}


void
xsens_mti_message_handler(carmen_xsens_global_quat_message *xsens_mti)
{
	StampedOrientation data;
	double *quat = xsens_mti->quat_data.m_data;
	Transform new_rotation;
	Quaternion q(quat[1], quat[2], quat[3], quat[0]);
	Matrix3x3(q).getEulerYPR(data.orientation.yaw, data.orientation.pitch, data.orientation.roll);
	data.time = xsens_mti->timestamp;
	xsens_queue.push_back(data);
}


void
robot_ackerman_velocity_handler(carmen_robot_ackerman_velocity_message *robot_ackerman_velocity_message)
{
	StampedOdometry data;

	data.v = robot_ackerman_velocity_message->v;
	data.phi = robot_ackerman_velocity_message->phi;
	data.time = robot_ackerman_velocity_message->timestamp;

	raw_command.push_back(data);
}


void
subscribe_to_ipc_messages()
{
	carmen_gps_xyz_subscribe_message(NULL,
		(carmen_handler_t) gps_xyz_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_xsens_subscribe_xsens_global_quat_message(NULL,
		(carmen_handler_t) xsens_mti_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_robot_ackerman_subscribe_velocity_message(NULL,
		(carmen_handler_t) robot_ackerman_velocity_handler,
		CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char **argv)
{
	if (argc < 2)
	{
		printf("Use %s <output-file>\n", argv[0]);
		exit(0);
	}

	output_filename = argv[1];
	output_file = fopen(output_filename, "w");
	if (output_file == NULL)
		exit(printf("Unable to open the file '%s'\n", output_filename));

	carmen_randomize(&argc, &argv);
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_grab_data_for_calibration);

	subscribe_to_ipc_messages();
	carmen_ipc_dispatch();

	return 0;
}


#include <vector>
#include <carmen/carmen.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/localize_ackerman_interface.h>

using namespace std;
vector<pair<carmen_point_t, double> > poses_array;

void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("graph_slam_correct: disconnected.\n");

		exit(0);
	}
}


void
graphslam_load_corrected_poses(char *filename)
{
	double timestamp;
	carmen_point_t point;
	FILE *graphslam_file = fopen(filename, "r");

	while(!feof(graphslam_file))
	{
		fscanf(graphslam_file, "%lf %lf %lf %lf\n",
			&point.x, &point.y, &point.theta, &timestamp);

		poses_array.push_back(pair<carmen_point_t, double>(point, timestamp));
	}

	fclose(graphslam_file);
	printf("Load complete.\n");
}


int
find_more_synchronized_pose(double timestamp)
{
	uint i;
	int index_min_time_diff;
	double min_time_diff, time_diff;

	min_time_diff = DBL_MAX;
	index_min_time_diff = -1;

	for (i = 0; i < poses_array.size(); i++)
	{
		time_diff = timestamp - poses_array[i].second;

		if ((time_diff < min_time_diff) && (time_diff > 0))
		{
			min_time_diff = time_diff;
			index_min_time_diff = i;
		}
	}

	if (index_min_time_diff >= ((int) poses_array.size() - 1))
		return (-1);
	else
		return (index_min_time_diff);
}


void
assembly_and_publish_globalpos_message(carmen_point_t point, double timestamp, double v, double phi)
{
	carmen_localize_ackerman_globalpos_message message;
	memset(&message, 0, sizeof(message));

	message.phi = phi;
	message.v = v;
	message.converged = 1;
	message.pose.orientation.yaw = point.theta;
	message.pose.position.x = point.x;
	message.pose.position.y = point.y;
	message.velocity.x = v;

	message.globalpos.x = point.x;
	message.globalpos.y = point.y;
	message.globalpos.theta = point.theta;

	message.host = carmen_get_host();
	message.timestamp = timestamp;

	carmen_localize_ackerman_publish_globalpos_message(&message);
}


void
assembly_and_publish_fused_odometry_message(carmen_point_t point, double timestamp, double v, double phi)
{
	carmen_fused_odometry_message message;
	memset(&message, 0, sizeof(message));
	memset(&message.pose, 0, sizeof(message.pose));
	memset(&message.velocity, 0, sizeof(message.velocity));
	memset(&message.angular_velocity, 0, sizeof(message.angular_velocity));

	message.phi = phi;
	message.velocity.x = v;
	message.pose.orientation.yaw = point.theta;
	message.pose.position.x = point.x;
	message.pose.position.y = point.y;

	message.gps_position_at_turn_on.x = poses_array[0].first.x;
	message.gps_position_at_turn_on.y = poses_array[0].first.y;
	message.gps_position_at_turn_on.z = 0.0;

	message.host = carmen_get_host();
	message.timestamp = timestamp;

	carmen_fused_odometry_publish_message(&message);
}


void
assembly_and_publish_fused_odometry_particles(carmen_point_t point, double timestamp, double v, double phi)
{
	carmen_fused_odometry_particle_message message;

	memset(&message, 0, sizeof(message));
	memset(&message.pose, 0, sizeof(message.pose));
	memset(&message.velocity, 0, sizeof(message.velocity));
	memset(&message.angular_velocity, 0, sizeof(message.angular_velocity));

	message.phi = phi;
	message.velocity.x = v;
	message.pose.orientation.yaw = point.theta;
	message.pose.position.x = point.x;
	message.pose.position.y = point.y;

	message.gps_position_at_turn_on.x = poses_array[0].first.x;
	message.gps_position_at_turn_on.y = poses_array[0].first.y;
	message.gps_position_at_turn_on.z = 0.0;

	message.host = carmen_get_host();
	message.timestamp = timestamp;

	message.num_particles = 1;
	message.particle_pos = (carmen_vector_3D_t*) calloc (1, sizeof(carmen_vector_3D_t*));
	message.weights = (double*) calloc (1, sizeof(double));
	message.weight_type = 0;

	message.particle_pos[0].x = point.x;
	message.particle_pos[0].y = point.y;
	message.particle_pos[0].z = 0.0;
	message.weights[0] = 1.0;

	carmen_fused_odometry_publish_particles(&message);

	free(message.particle_pos);
	free(message.weights);
}


void
base_ackerman_message_handler(carmen_base_ackerman_odometry_message *message)
{
	if ((poses_array.size() > 0) && (message->v > 0.1))
	{
		int index;
		carmen_point_t pose;

		index = find_more_synchronized_pose(message->timestamp);

		if (index < 0)
			return;

		pose = poses_array[index].first;
		double timestamp = poses_array[index].second;

		// Set the if condition to 1 to use the Ackerman motion model to predict the vehicle's pose 
		// in the base_ackerman timestamp. 
		if (0) 
		{
			double dt = message->timestamp - poses_array[index].second;

			if (dt < 0)
				exit(printf("dt: %lf\n", dt));

			//printf("pose.x = %lf, pose.y = %lf, dt = %lf, message->timestamp = %lf, message->v = %lf\n", pose.x, pose.y, dt, message->timestamp, message->v);
			dt = 0;
			pose.x = pose.x + dt * message->v * cos(pose.theta);
			pose.y = pose.y + dt * message->v * sin(pose.theta);
			pose.theta = pose.theta + dt * (message->v / 2.625 /* L */) * tan(message->phi);
			pose.theta = carmen_normalize_theta(pose.theta);
			timestamp = message->timestamp;
		}
		
		assembly_and_publish_fused_odometry_message(pose, timestamp, message->v, message->phi);
		assembly_and_publish_fused_odometry_particles(pose, timestamp, message->v, message->phi);
	}
}


static void 
define_fused_odometry_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_FUSED_ODOMETRY_NAME, IPC_VARIABLE_LENGTH, CARMEN_FUSED_ODOMETRY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FUSED_ODOMETRY_NAME);

	err = IPC_defineMsg(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FUSED_ODOMETRY_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FUSED_ODOMETRY_PARTICLE_NAME);

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME,	IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_ODOMETRY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROBOT_ACKERMAN_FRONTLASER_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_SENSOR_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME);
}


int 
main(int argc, char **argv) 
{
	if (argc < 2)
		exit(printf("Use %s <arquivo-poses.txt>\n", argv[0]));

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	define_fused_odometry_messages();

	signal(SIGINT, shutdown_module);
	graphslam_load_corrected_poses(argv[1]);

	carmen_base_ackerman_subscribe_odometry_message(NULL,
		(carmen_handler_t) base_ackerman_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();
	return(0);
}

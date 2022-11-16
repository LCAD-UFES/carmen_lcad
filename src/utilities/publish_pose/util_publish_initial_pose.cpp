#include <carmen/carmen.h>
#include <carmen/playback_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>


carmen_point_t pose;
double wait_time = 4.;
bool wait_play = false;


carmen_point_t
publish_starting_pose(carmen_point_t pose)
{
	carmen_point_t std;
	std.x = 0.001;
	std.y = 0.001;
	std.theta = carmen_degrees_to_radians(0.01);
	double theta_semi_trailer = pose.theta;

	carmen_localize_ackerman_initialize_gaussian_command(pose, std, theta_semi_trailer);

	return pose;
}


void
define_messages()
{
	IPC_RETURN_TYPE err;

	carmen_localize_ackerman_define_globalpos_messages();

	err = IPC_defineMsg(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FUSED_ODOMETRY_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FUSED_ODOMETRY_PARTICLE_NAME);
}


void
send_pose()
{
	printf("publishing %lf %lf %lf\n", pose.x, pose.y, pose.theta);
	for (int i = 0; i < 2; i++)
	{
		publish_starting_pose(pose);
		carmen_ipc_sleep(0.1);
	}
	printf("programa concluido normalmente. tecle qualquer tecla para terminar.\n");
	fflush(stdout);
}


void
playback_handler(carmen_playback_info_message *msg)
{
	static double first_timestamp = -1, last_timestamp = 0.0;
	if (first_timestamp < 0)
	{
		first_timestamp = msg->message_timestamp;
		last_timestamp = msg->message_timestamp;
		return;
	}

	if ((msg->message_timestamp - last_timestamp) > 0.3) // manual
	{
		first_timestamp = msg->message_timestamp;
		last_timestamp = msg->message_timestamp;
		return;
	}
	
	if ((msg->message_timestamp - first_timestamp) >= wait_time)
	{
		send_pose();
		carmen_unsubscribe_playback_info_message((carmen_handler_t) playback_handler);

		while (1)
			sleep(10);
	}
	last_timestamp = msg->message_timestamp;
}


int
main(int argc, char **argv)
{
	if (argc < 4)
	{
		printf("Use %s <x> <y> <theta>\n"
		 	   "    <OPTIONAL wait_time in seconds> time to wait before publishing the initial pose\n"
			   "    <OPTIONAL --wait-playback> wait playback play to start. wait_time is mandatory in this case>\n"
				"\n", argv[0]);
		exit(-1);
	}
	/* NOTE
	poses can be obtained from the tmp/poses_opt.txt file.
	the waiting time can be obtained from the difference of the initial timestamp by the timestamp of the desired pose.
	*/
	pose.x = atof(argv[1]);
	pose.y = atof(argv[2]);
	pose.theta = atof(argv[3]);

	if (argc >= 5)
		wait_time = atof(argv[4]);
	if (argc == 6 && (argv[5][0] == '-'))
		wait_play = true;

	carmen_ipc_initialize(argc, argv);
	define_messages();

	carmen_param_check_version(argv[0]);
	if (wait_play)
	{
		carmen_subscribe_playback_info_message(NULL, (carmen_handler_t) playback_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_ipc_dispatch();
	}
	else
	{
		sleep((int) wait_time);
		send_pose();
		while (1)
			sleep(10);
	}

	return 0;
}

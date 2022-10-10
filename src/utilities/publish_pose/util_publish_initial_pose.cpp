#include <carmen/carmen.h>
#include <carmen/playback_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>

bool wait_play = false;

carmen_point_t pose;
double beta = 0.0;
int time_value = 4;


carmen_point_t
publish_starting_pose(carmen_point_t pose, double beta)
{
	carmen_point_t std;

	std.x = 0.001;
	std.y = 0.001;
	std.theta = carmen_degrees_to_radians(0.01);

	carmen_localize_ackerman_initialize_gaussian_command(pose, std, beta);

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


int
build_and_send_pose()
{
	printf("sleeping %ds\n", time_value);
	sleep(time_value);
	printf("publishing %lf %lf %lf\n", pose.x, pose.y, pose.theta);
	for (int i = 0; i < 2; i++)
	{
		publish_starting_pose(pose, beta);
		carmen_ipc_sleep(0.1);
	}
	printf("programa concluido normalmente. tecle qualquer tecla para terminar.\n");
	fflush(stdout);

	return 0;
}


void playback_handler(carmen_playback_info_message *msg)
{
	static int wait_playback = 0;
	
	// wait for play: first message, then second message when plays
	wait_playback ++;
	printf("playback message timestamp %lfs\n", msg->message_timestamp);

	if (wait_playback == 2)
	{
		carmen_unsubscribe_playback_info_message((carmen_handler_t) playback_handler);
		build_and_send_pose();
	}
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

	if (argc >= 5)
	{
		time_value = atoi(argv[4]);
		if (argc == 6)
		{
			if (argv[5][0] == '-')
				wait_play = true;
			else
				beta = atof(argv[5]); // old code
		}
	}

	pose.x = atof(argv[1]);
	pose.y = atof(argv[2]);
	pose.theta = atof(argv[3]);

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
		build_and_send_pose();
		while (1)
			sleep(10); // Para não morrer nunca e não gastar CPU
	}

	return 0;
}

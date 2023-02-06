#include <carmen/carmen.h>
#include <carmen/rddf_interface.h>


void
publish_final_goal(carmen_robot_and_trailers_pose_t pose)
{
	carmen_robot_and_trailers_pose_t pose_with_beta;
	pose_with_beta.x = pose.x;
	pose_with_beta.y = pose.y;
	pose_with_beta.theta = pose.theta;
	pose_with_beta.num_trailers = pose.num_trailers;
	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		pose_with_beta.trailer_theta[z] = pose.trailer_theta[z];
	carmen_rddf_publish_end_point_message(50, pose_with_beta);
}


void
define_messages()
{
	IPC_RETURN_TYPE err;

    err = IPC_defineMsg(CARMEN_RDDF_END_POINT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_END_POINT_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_END_POINT_MESSAGE_NAME);
}


int
main(int argc, char **argv)
{
	carmen_robot_and_trailers_pose_t pose;
	int time = 4;

	if (argc < 4)
	{
		printf("Use %s <x> <y> <theta> <OPTIONAL wait_time in seconds>\n "
				"Time to wait before publishing the final goal\n", argv[0]);
		exit(-1);
	}
	if (argc >= 5)
		time = atoi(argv[4]);

	pose.x = atof(argv[1]);
	pose.y = atof(argv[2]);
	pose.theta = atof(argv[3]);
	pose.trailer_theta[0] = pose.theta;

	if (argc == 6)
		pose.trailer_theta[0] = atof(argv[5]);

	carmen_ipc_initialize(argc, argv);
	define_messages();

	sleep(time);
	for (int i = 0; i < 2; i++)
	{
		publish_final_goal(pose);
		carmen_ipc_sleep(0.1);
	}

	while (1)
		sleep(10); // Para não morrer nunca e não gastar CPU

	return 0;
}


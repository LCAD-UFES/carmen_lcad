#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>


carmen_point_t
publish_starting_pose(carmen_point_t pose)
{
	carmen_point_t std;

	std.x = 0.001;
	std.y = 0.001;
	std.theta = carmen_degrees_to_radians(0.01);

	carmen_localize_ackerman_initialize_gaussian_command(pose, std);

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
main(int argc, char **argv)
{
	carmen_point_t pose;
	int time = 4;

	if (argc < 4)
	{
		printf("Use %s <x> <y> <theta> <OPTIONAL wait_time in seconds>\n "
				"Time to wait before publishing the initial pose\n", argv[0]);
		exit(-1);
	}
	if (argc == 5)
		time = atoi(argv[4]);

	pose.x = atof(argv[1]);
	pose.y = atof(argv[2]);
	pose.theta = atof(argv[3]);

	carmen_ipc_initialize(argc, argv);

	sleep(time);
	publish_starting_pose(pose);

	while (1)
		sleep(10); // Para não morrer nunca e não gastar CPU

	return 0;
}


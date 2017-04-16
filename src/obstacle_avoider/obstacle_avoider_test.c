#include <carmen/carmen.h>
#include <carmen/motion_planner.h>


typedef double (*MotionControlFunction)(double v, double w, double t);


static carmen_ackerman_motion_command_t motion_commands_vector[NUM_MOTION_COMMANDS_PER_VECTOR];


void
signal_handler(int signo __attribute__ ((unused)) )
{
	carmen_ipc_disconnect();

	exit(0);
}


void
build_trajectory_stop_smooth_trajectory_phi() //This function allows to create a trapezium shaped motion command
{
	for (int i = 0; i < 50; i++)
	{
		motion_commands_vector[i].v = 4.0;
		motion_commands_vector[i].phi = 0.0;//t * (max_phi / t1);
		motion_commands_vector[i].time = 0.1;//delta_t;
		//printf("i = %d, NUM_MOTION_COMMANDS_PER_VECTOR = %d\n", i, NUM_MOTION_COMMANDS_PER_VECTOR);
	}

//	printf("mandou\n");
	carmen_robot_ackerman_publish_motion_command(motion_commands_vector, NUM_MOTION_COMMANDS_PER_VECTOR, carmen_get_time());
}


void
timer_handler()
{
//	static int first_time = 1;

//	if (first_time)
//	{
		build_trajectory_stop_smooth_trajectory_phi();
//		first_time = 0;
//	}
}


void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_FMT);

	carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME);
}


int
main(int argc, char **argv)
{
	signal(SIGINT, signal_handler);

	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	define_messages();

	carmen_ipc_addPeriodicTimer(1.0, timer_handler, NULL);

	carmen_ipc_dispatch();

	return 0;
}

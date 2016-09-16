/*********************************************************
	---  Moving Objects Simulator Module ---
**********************************************************/

#include <carmen/carmen.h>
#include <carmen/localize_ackerman_core.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
void static
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("moving_objects_simulator: disconnected.\n");

		exit(0);
	}
}

void
localize_ackerman_init_handler(carmen_localize_ackerman_initialize_message localize_ackerman_init_message)
{
	double x_pos, y_pos;
	printf("num of modes %d\n",localize_ackerman_init_message.num_modes);
	if(localize_ackerman_init_message.num_modes > 0)
	{
		x_pos = localize_ackerman_init_message.mean[0].x;
		y_pos = localize_ackerman_init_message.mean[0].y;
		printf("Robot position: %lf %lf\n", x_pos, y_pos);
	}

}


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////

static int
read_parameters(int argc, char **argv)
{
	if (argc > 1)
	{
		printf("%s\n",argv[1]);
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
int
main(int argc, char **argv)
{
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Initialize all the relevant parameters */
	read_parameters(argc, argv);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Subscribe to sensor and filter messages */
	carmen_localize_ackerman_subscribe_initialize_message(NULL, (carmen_handler_t) localize_ackerman_init_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return (0);
}

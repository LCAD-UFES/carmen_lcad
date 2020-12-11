#include <stdio.h>
#include <carmen/carmen.h>
#include <carmen/can_dump_interface.h>


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
can_dump_message_handler(carmen_can_dump_can_line_message *message)
{
	printf("%s - %lf\n", message->can_line, message->timestamp);
}


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("can_view: disconnected.\n");

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////



int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	carmen_can_dump_define_can_line_message();

	carmen_can_dump_subscribe_can_line_message(NULL, (carmen_handler_t) can_dump_message_handler, CARMEN_SUBSCRIBE_ALL);
	signal(SIGINT, shutdown_module);

	carmen_ipc_dispatch();

	return (0);
}

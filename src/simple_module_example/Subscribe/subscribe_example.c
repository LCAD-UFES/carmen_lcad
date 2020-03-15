#include <carmen/carmen.h>
#include "../Publish/publish_example_messages.h"



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
carmen_simple_handler(carmen_string_example_message *current_message)
{
	printf("Mensagem recebida: %s\n",current_message->content_message.message);

}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        printf("Subscribe_example: Disconnected.\n");
        exit(0);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Subscribes                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
carmen_simple_subscribe_motion_command(carmen_string_example_message *message,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_STRING_EXAMPLE_MESSAGE_NAME, CARMEN_STRING_EXAMPLE_MESSAGE_FMT,
                	   message, sizeof(carmen_string_example_message), carmen_simple_handler, subscribe_how);
}

///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc , char **argv)
{
	carmen_ipc_initialize(argc, argv);

	signal(SIGINT, shutdown_module);

	printf("Aguardando mensagem\n");

	carmen_simple_subscribe_motion_command(NULL, (carmen_handler_t) carmen_simple_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return 0;
}

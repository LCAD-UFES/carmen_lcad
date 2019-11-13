#include <carmen/carmen.h>
#include "../Publish/publish_example_messages.h"

int camera;

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
subscribe_messages()
{
	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
}

int
read_parameters(int argc, char **argv)
{
	camera = atoi(argv[1]);
}


int
main(int argc , char **argv)
{
	carmen_ipc_initialize(argc, argv);

	signal(SIGINT, shutdown_module);

	read_parameters();

	printf("Aguardando mensagem\n");

	subscribe_messages();

	carmen_ipc_dispatch();

	return 0;
}

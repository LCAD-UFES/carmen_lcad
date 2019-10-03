#include <carmen/carmen.h>
#include "publish_example_messages.h"


void
example_define_messages()
{

	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_STRING_EXAMPLE_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_STRING_EXAMPLE_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_STRING_EXAMPLE_MESSAGE_NAME);

}

void
carmen_publish_example_publish_command_message(carmen_string_example_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData((char *) CARMEN_STRING_EXAMPLE_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", (char *) CARMEN_STRING_EXAMPLE_MESSAGE_FMT);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        printf("Publish_example: Disconnected.\n");
        exit(0);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publish                                                                                   //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
carmen_mount_simple_message()
{
	carmen_string_example_message main_message;
	carmen_string_example main_content;
	main_content.message = "Mensagem enviada através do módulo publish_example";

	printf("Enviando mensagem: %s\n",main_content.message);

	main_message.num_message = 1;
	main_message.content_message = main_content;
	main_message.timestamp = carmen_get_time();
	main_message.host = carmen_get_host();

	carmen_publish_example_publish_command_message(&main_message);
}

///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc , char **argv)
{
	carmen_ipc_initialize(argc, argv);

	signal(SIGINT, shutdown_module);

	example_define_messages();

	carmen_mount_simple_message();

	carmen_ipc_dispatch();

	return 0;
}

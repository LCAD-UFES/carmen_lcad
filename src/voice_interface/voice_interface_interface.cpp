/*
 * voice_interface_interface.c
 *
 *  Created on: Aug 24, 2017
 *      Author: alberto
 */

#include <carmen/voice_interface_interface.h>


void
carmen_voice_interface_subscribe_command_message(carmen_voice_interface_command_message *message, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char *) CARMEN_VOICE_INTERFACE_COMMAND_MESSAGE_NAME, (char *) CARMEN_VOICE_INTERFACE_COMMAND_MESSAGE_FMT, message,
			sizeof(carmen_voice_interface_command_message), handler, subscribe_how);
}


void
carmen_voice_interface_unsubscribe_command_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char *) CARMEN_VOICE_INTERFACE_COMMAND_MESSAGE_NAME, handler);
}


void
carmen_voice_interface_publish_command_message(carmen_voice_interface_command_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData((char *) CARMEN_VOICE_INTERFACE_COMMAND_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", (char *) CARMEN_VOICE_INTERFACE_COMMAND_MESSAGE_FMT);
}


void
carmen_voice_interface_define_command_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg((char *) CARMEN_VOICE_INTERFACE_COMMAND_MESSAGE_NAME, IPC_VARIABLE_LENGTH, (char *) CARMEN_VOICE_INTERFACE_COMMAND_MESSAGE_FMT);
	carmen_test_ipc_exit(err, (char *) "Could not define", (char *) CARMEN_VOICE_INTERFACE_COMMAND_MESSAGE_NAME);
}

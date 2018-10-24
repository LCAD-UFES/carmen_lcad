/*
 * voice_interface_interface.c
 *
 *  Created on: Aug 24, 2017
 *      Author: alberto
 */

#include <carmen/voice_interface_interface.h>
#include "voice_interface.h"


void
carmen_voice_interface_subscribe_can_line_message(carmen_voice_interface_can_line_message *message, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_NAME, CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_FMT, message,
			sizeof(carmen_voice_interface_can_line_message), handler, subscribe_how);
}


void
carmen_voice_interface_unsubscribe_can_line_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_NAME, handler);
}


void
carmen_voice_interface_publish_can_line_message(carmen_voice_interface_can_line_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_FMT);
}


void
carmen_voice_interface_define_can_line_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_NAME);
}


char *
carmen_voice_interface_speak(char *speech)
{
	FILE *list_of_speechs = fopen(LIST_OF_SPEECHS_FILE, "r");
	if (list_of_speechs == NULL)
	{
		char *voice_interface_error = init_voice();
		if (voice_interface_error != NULL)
			return (voice_interface_error);

		char *speech_file_name = "speech00001.wav";
		if (speak(speech, speech_file_name) != 0)
		{
			finalize_voice();
			return ("Error: Could not load SPEAK function.\n");
		}

		list_of_speechs = fopen(LIST_OF_SPEECHS_FILE, "w");
		fprintf(list_of_speechs, "%s %s\n", speech_file_name, speech);

		finalize_voice();
	}

	return (NULL); // OK
}


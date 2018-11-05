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
	carmen_subscribe_message((char *) CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_NAME, (char *) CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_FMT, message,
			sizeof(carmen_voice_interface_can_line_message), handler, subscribe_how);
}


void
carmen_voice_interface_unsubscribe_can_line_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char *) CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_NAME, handler);
}


void
carmen_voice_interface_publish_can_line_message(carmen_voice_interface_can_line_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData((char *) CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", (char *) CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_FMT);
}


void
carmen_voice_interface_define_can_line_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg((char *) CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_NAME, IPC_VARIABLE_LENGTH, (char *) CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_FMT);
	carmen_test_ipc_exit(err, (char *) "Could not define", (char *) CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_NAME);
}

void
carmen_voice_interface_create_new_audio_file(FILE *list_of_speechs, char *speech, char *last_audio_name_used)
{
	int new_number = 0, length_of_name_speech = 6, length_of_number = 5;
	char speech_audio_name[MAX_SIZE_0F_AUDIO_STRING], last_number_used[length_of_number];

	strncpy(speech_audio_name, last_audio_name_used, length_of_name_speech);
	strncpy(last_number_used, last_audio_name_used+length_of_name_speech, length_of_number);

	new_number = atoi (last_number_used);
	new_number++;
	sprintf (speech_audio_name, "%s%.5d", speech_audio_name, new_number);

	init_voice();
	speak((char *)speech, speech_audio_name);
	finalize_voice();
	fprintf (list_of_speechs, "%s %s\n", speech_audio_name,speech);
}

char *
carmen_voice_interface_speak(char *speech)
{
	char *CARMEN_HOME ;
	CARMEN_HOME = (char *)malloc (DEFAULT_SIZE_COMMAND_LINE * sizeof (char ));

	char *CARMENHOME;
	CARMENHOME = (char *)malloc (DEFAULT_SIZE_COMMAND_LINE * sizeof (char ));
	CARMENHOME =  getenv ("CARMEN_HOME");

	strcat (CARMEN_HOME, CARMENHOME);
	strcat (CARMEN_HOME, VOICE_SPEECHS_PATH);
	strcat (CARMEN_HOME, LIST_OF_SPEECHS_FILE);

	FILE *list_of_speechs = fopen(CARMEN_HOME, "r");

	if (list_of_speechs == NULL)
	{
		char *voice_interface_error = init_voice();
		if (voice_interface_error != NULL)
			return (voice_interface_error);

		char *speech_file_name = (char *) "speech00001";
		if (speak(speech, speech_file_name) != 0)
		{
			finalize_voice();
			return ((char *) "Error: Could not load SPEAK function.\n");
		}

		list_of_speechs = fopen (CARMEN_HOME, "w");
		fprintf (list_of_speechs, "%s %s\n", speech_file_name, (char *)speech);
		fclose (list_of_speechs);
		finalize_voice();
	}
	else
	{

		int speech_string_matched = 0;
		char audio_string[MAX_SIZE_0F_AUDIO_STRING];
		char speakers_command[DEFAULT_SIZE_COMMAND_LINE];
		char speech_file_name[DEFAULT_SIZE_NAME_AUDIO];
		char speech_phrase[MAX_SIZE_0F_AUDIO_STRING];

		strncpy (speech_phrase, speech, MAX_SIZE_0F_AUDIO_STRING);
		fseek (list_of_speechs, 0, SEEK_SET);
		do {
			fscanf (list_of_speechs, "%s %[^\n] ", speech_file_name, audio_string);
			if (strcmp (speech_phrase, audio_string) == 0)
			{

				strcat (CARMENHOME, VOICE_SPEECHS_PATH);
				sprintf (speakers_command, "aplay %s%s", CARMENHOME, speech_file_name);
				system (speakers_command);
				speech_string_matched = 1;
			}
		} while (!feof (list_of_speechs) && (speech_string_matched == 0));
		fclose (list_of_speechs);

		if (speech_string_matched == 0)
		{
			FILE *list_of_speechs = fopen(CARMEN_HOME, "a");
			carmen_voice_interface_create_new_audio_file(list_of_speechs, speech_phrase, speech_file_name);
			fclose(list_of_speechs);
		}
	}

	return (NULL); // OK
}


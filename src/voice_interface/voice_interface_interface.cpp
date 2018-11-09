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
	int new_number = 0;
	char speech_audio_name[MAX_SIZE_0F_AUDIO_STRING], last_number_used[LENGTH_NUMBER_AS_STRING];

	strncpy (speech_audio_name, last_audio_name_used, LENGTH_SPEECH_WORD);
	strncpy (last_number_used, last_audio_name_used + LENGTH_SPEECH_WORD, LENGTH_NUMBER_AS_STRING);

	new_number = atoi (last_number_used);
	new_number++;
	sprintf (speech_audio_name, "%s%.5d", speech_audio_name, new_number);

	init_voice();
	speak((char *)speech, speech_audio_name);
	finalize_voice();
	fprintf (list_of_speechs, "%s %s\n", speech_audio_name,speech);
	printf("Audio content created. \n");
}

char *
carmen_voice_interface_speak(char *speech)
{
	char *list_of_speechs_path ;
	list_of_speechs_path = (char *)malloc (MAX_SIZE_COMMAND_LINE * sizeof (char ));

	char *CARMEN_HOME;
	CARMEN_HOME = (char *)malloc (MAX_SIZE_COMMAND_LINE * sizeof (char ));
	CARMEN_HOME =  getenv ("CARMEN_HOME");

	sprintf(list_of_speechs_path, "%s%s%s", CARMEN_HOME, VOICE_SPEECHS_PATH, LIST_OF_SPEECHS_FILE);

	FILE *list_of_speechs = fopen(list_of_speechs_path, "r");

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

		list_of_speechs = fopen (list_of_speechs_path, "w");
		fprintf (list_of_speechs, "%s %s\n", speech_file_name, (char *)speech);
		fclose (list_of_speechs);
		finalize_voice();
	}
	else
	{
		int speech_string_matched = 0;
		char audio_string[MAX_SIZE_0F_AUDIO_STRING], speech_phrase[MAX_SIZE_0F_AUDIO_STRING];
		char speech_file_name[LENGTH_SPEECH_WORD + LENGTH_NUMBER_AS_STRING];

		strncpy (speech_phrase, speech, MAX_SIZE_0F_AUDIO_STRING);

		do {
			fscanf (list_of_speechs, "%s %[^\n]", speech_file_name, audio_string);

			if (strcmp (speech_phrase, audio_string) == 0)
			{
				char system_command[MAX_SIZE_COMMAND_LINE];

				sprintf (system_command, "aplay %s%s%s", CARMEN_HOME, VOICE_SPEECHS_PATH, speech_file_name);
				system (system_command);
				speech_string_matched = 1;
			}
		} while (!feof (list_of_speechs) && (speech_string_matched == 0));
		fclose (list_of_speechs);

		if (speech_string_matched == 0)
		{
			FILE *list_of_speechs = fopen (list_of_speechs_path, "a");
			carmen_voice_interface_create_new_audio_file(list_of_speechs, speech_phrase, speech_file_name);
			fclose (list_of_speechs);
		}
	}
	free(list_of_speechs_path);
	return (NULL); // OK
}

char *
carmen_voice_interface_listen(){

	char *voice_interface_error = init_voice();
			if (voice_interface_error != NULL)
				return (voice_interface_error);

	const char * words_said = listen();
	printf("\n");
	printf("Message: '%s'\n", words_said);
	printf("\n");
	finalize_voice();

	return NULL;
}

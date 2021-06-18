/*
 * voice_interface_interface.c
 *
 *  Created on: Aug 24, 2017
 *      Author: alberto
 */

#include <carmen/voice_interface_interface.h>
#include "voice_interface.h"


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


void
carmen_voice_interface_create_new_audio_file(FILE *list_of_speechs, char *speech, char *last_audio_name_used)
{
	int new_number = 0;
	char last_number_used[LENGTH_NUMBER_AS_STRING + 1];

	strcpy(last_number_used, last_audio_name_used + strlen("speech"));

	new_number = atoi (last_number_used);
	new_number++;
	char speech_file_name[LENGTH_SPEECH_WORD + LENGTH_NUMBER_AS_STRING + 1];
	sprintf (speech_file_name, "%s%.5d", "speech", new_number);

	if (speak(speech, speech_file_name) == 0)
	{
		fprintf (list_of_speechs, "%s %s\n", speech_file_name, speech);
		printf("Audio content created.\n");
	}
	else
		printf("Unable to create audio content for speech '%s'.\n", speech);
}


char *
carmen_voice_interface_speak(char *speech)
{
	char *carmen_home = getenv ("CARMEN_HOME");

	char list_of_speechs_filename[MAX_SIZE_COMMAND_LINE];
	sprintf(list_of_speechs_filename, "%s%s%s", carmen_home, VOICE_SPEECHS_PATH, LIST_OF_SPEECHS_FILE);

	FILE *list_of_speechs = fopen(list_of_speechs_filename, "r");
	if (list_of_speechs == NULL)
	{
		char *speech_file_name = (char *) "speech00001";
		if (speak(speech, speech_file_name) == 0)
		{
			list_of_speechs = fopen(list_of_speechs_filename, "w");
			fprintf(list_of_speechs, "%s %s\n", speech_file_name, speech);
			printf("Audio content created.\n");
			fclose(list_of_speechs);
		}
		else
			printf("Unable to create audio content for speech '%s'.\n", speech);
	}
	else
	{
		int speech_string_matched = 0;
		int speech_file_name_size = strlen("speech") + strlen("00001");
		char speech_file_name_and_speech_saved[speech_file_name_size + MAX_SIZE_0F_AUDIO_STRING];
		char speech_saved[MAX_SIZE_0F_AUDIO_STRING];
		char speech_file_name[speech_file_name_size + 1];

		while (fgets(speech_file_name_and_speech_saved, speech_file_name_size + MAX_SIZE_0F_AUDIO_STRING, list_of_speechs))
		{
			speech_file_name_and_speech_saved[speech_file_name_size] = '\0';
			strcpy(speech_file_name, speech_file_name_and_speech_saved);
			strcpy(speech_saved, speech_file_name_and_speech_saved + speech_file_name_size + 1);
			if (speech_saved[strlen(speech_saved) - 1] == '\n')
				speech_saved[strlen(speech_saved) - 1] = '\0';
			if (speech[strlen(speech) - 1] == '\n')
				speech[strlen(speech) - 1] = '\0';

			if (strcmp(speech_saved, speech) == 0)
			{
				char system_command[MAX_SIZE_COMMAND_LINE];

				sprintf(system_command, "aplay %s%s%s", carmen_home, VOICE_SPEECHS_PATH, speech_file_name);
				system(system_command);
				speech_string_matched = 1;
				break;
			}
		}
		fclose (list_of_speechs);

		if (speech_string_matched == 0)
		{
			FILE *list_of_speechs = fopen (list_of_speechs_filename, "a");
			carmen_voice_interface_create_new_audio_file(list_of_speechs, speech, speech_file_name);
			fclose(list_of_speechs);
		}
	}

	return (NULL); // OK
}


char *
carmen_voice_interface_listen()
{
	static char words_said[MAX_LISTENDED_STRING_SIZE];

	listen(words_said);

	return (words_said);
}

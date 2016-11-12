/*
 * voice.c
 *
 *  Created on: Mar 27, 2013
 *      Author: lcad
 *
 * TODO: Criar uma forma de checar se a mesma mensagem nao esta sendo repetida sempre.
 */
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include "voice.h"

double last_message_published_time = 0;

void
carmen_voice_send_alert(char *message, int min_time_between_messages, char *language)
{
	static int start_speaking = 0;
	double current_time = time(NULL);

	if (fabs(current_time - last_message_published_time) > min_time_between_messages)
	{
		void* user_data = NULL;
		char *path = NULL;
		int Buflength = 500, Options = 0;
		unsigned int Size,position = 0, end_position = 0, flags = espeakCHARS_AUTO, *unique_identifier = 0;

		espeak_POSITION_TYPE position_type = POS_CHARACTER;
		espeak_AUDIO_OUTPUT output;

		char *Voice = language;
		Size = strlen(message) + 1;

		output = AUDIO_OUTPUT_PLAYBACK;

		if (espeak_IsPlaying())
			return;

		if (start_speaking && !espeak_IsPlaying())
		{
			last_message_published_time = time(NULL);
			start_speaking = 0;
		}

		printf("saying '%s'\n", message);
		espeak_Terminate();
		espeak_Initialize(output, Buflength, path, Options);
		espeak_SetVoiceByName(Voice);
		espeak_Synth(message, Size, position, position_type, end_position, flags, unique_identifier, user_data);
		last_message_published_time = time(NULL);
		start_speaking = 1;
	}
}




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

#define MIN_TIME_BETWEEN_MESSAGES 1.0 // essa variavel define o intervalo minimo entre as mensagens para que o robo nao fique falando toda hora
double last_message_published_time = 0;

void
carmen_voice_send_alert(char *message)
{
	static int start_playing = 0;
	double current_time = time(NULL);

	if (fabs(current_time - last_message_published_time) > MIN_TIME_BETWEEN_MESSAGES)
	{
		void* user_data = NULL;
		char *path = NULL;
		int Buflength = 500, Options = 0;
		unsigned int Size,position = 0, end_position = 0, flags = espeakCHARS_AUTO, *unique_identifier = 0;

		espeak_POSITION_TYPE position_type = POS_CHARACTER;
		espeak_AUDIO_OUTPUT output;

		char Voice[] = {"pt+f3"};
		Size = strlen(message) + 1;

		output = AUDIO_OUTPUT_PLAYBACK;

		if (start_playing && !espeak_IsPlaying())
		{
			espeak_Terminate();
			start_playing = 0;
		}
		else if (start_playing && espeak_IsPlaying())
			return;

		printf("saying '%s'\n", message);
		espeak_Initialize(output, Buflength, path, Options);
		espeak_SetVoiceByName(Voice);
		espeak_Synth(message, Size, position, position_type, end_position, flags, unique_identifier, user_data);
		start_playing = 1;

		last_message_published_time = time(NULL);
	}
}




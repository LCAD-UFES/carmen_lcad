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
carmen_voice_initialize(char *language)
{
	espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, 500, NULL, 0);
	espeak_SetVoiceByName(language);
}


void
carmen_voice_send_alert(char *message, int min_time_between_messages, char *language)
{
	static int first = 1;

	if (first)
	{
		carmen_voice_initialize(language);
		first = 0;
	}

	/**
	 * The following code is a state machine. While the computer is speaking,
	 * the code do nothing, it just returns. If it ended speaking, we wait some time
	 * before start speaking again.
	 */
	static int started_speaking = 0;
	double current_time = time(NULL);

	unsigned int text_size;

	text_size = strlen(message) + 1;

	if (espeak_IsPlaying())
		return;

	if (started_speaking && !espeak_IsPlaying())
	{
		last_message_published_time = time(NULL);
		started_speaking = 0;
	}

	if (fabs(current_time - last_message_published_time) < min_time_between_messages)
		return;

	printf("saying '%s'\n", message);
	espeak_Synth(message, text_size, 0, POS_CHARACTER, 0, espeakCHARS_AUTO, NULL, NULL);
	started_speaking = 1;
}




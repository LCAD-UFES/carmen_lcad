/*
 * voice_interface_interface.h
 *
 *  Created on: Aug 24, 2017
 *      Author: alberto
 */

#ifndef SRC_VOICE_INTERFACE_VOICE_INTERFACE_INTERFACE_H_
#define SRC_VOICE_INTERFACE_VOICE_INTERFACE_INTERFACE_H_

#include <carmen/carmen.h>
#include <carmen/voice_interface_messages.h>
#include <Python.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#ifdef __cplusplus
extern "C"
{
#endif

#define LIST_OF_SPEECHS_FILE "list_of_speechs.txt"
#define VOICE_SPEECHS_PATH "/data/voice_interface_speechs/"
#define MAX_SIZE_0F_AUDIO_STRING 1000
#define MAX_SIZE_COMMAND_LINE 10000
#define LENGTH_SPEECH_WORD 6
#define LENGTH_NUMBER_AS_STRING 5

void
carmen_voice_interface_subscribe_can_line_message(carmen_voice_interface_can_line_message *message, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void
carmen_voice_interface_unsubscribe_can_line_message(carmen_handler_t handler);

void
carmen_voice_interface_publish_can_line_message(carmen_voice_interface_can_line_message *message);

void
carmen_voice_interface_define_can_line_message();

void
carmen_voice_interface_create_new_audio_file(FILE *list_of_speechs, char *speech, char *last_audio_name_used);

char *
carmen_voice_interface_speak(char *speech);

char *
carmen_voice_interface_listen();

#ifdef __cplusplus
}
#endif

#endif /* SRC_VOICE_INTERFACE_VOICE_INTERFACE_INTERFACE_H_ */

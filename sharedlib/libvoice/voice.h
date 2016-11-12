/*
 * voice.h
 *
 *  Created on: Mar 27, 2013
 *      Author: lcad
 */

#ifndef VOICE_H_
#define VOICE_H_

#include <espeak/speak_lib.h>
void carmen_voice_send_alert(char *message, int min_time_between_messages = 1, char *language = "pt+f3");

#endif /* VOICE_H_ */

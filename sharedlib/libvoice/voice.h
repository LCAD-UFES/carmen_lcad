/*
 * voice.h
 *
 *  Created on: Mar 27, 2013
 *      Author: lcad
 */

#ifndef VOICE_H_
#define VOICE_H_

#ifdef __cplusplus
extern "C" {
#endif

	#include <espeak/speak_lib.h>
	void carmen_voice_send_alert(char *message);

#ifdef __cplusplus
}
#endif
#endif /* VOICE_H_ */

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


#ifdef __cplusplus
extern "C"
{
#endif

#define LIST_OF_SPEECHS_FILE "$CARMEN_HOME/data/voice_interface_speechs/list_of_speechs.txt"


void
carmen_voice_interface_subscribe_can_line_message(carmen_voice_interface_can_line_message *message, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void
carmen_voice_interface_unsubscribe_can_line_message(carmen_handler_t handler);

void
carmen_voice_interface_publish_can_line_message(carmen_voice_interface_can_line_message *message);

void
carmen_voice_interface_define_can_line_message();

char *
carmen_voice_interface_speak(char *speech);

#ifdef __cplusplus
}
#endif

#endif /* SRC_VOICE_INTERFACE_VOICE_INTERFACE_INTERFACE_H_ */

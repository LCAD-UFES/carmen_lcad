/*
 * voice_interface_messages.h
 *
 *  Created on: Aug 24, 2017
 *      Author: alberto
 */

#ifndef SRC_VOICE_INTERFACE_MESSAGES_H_
#define SRC_VOICE_INTERFACE_MESSAGES_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Message Struct Example */
typedef struct
{
	char *can_line;
	double timestamp;
	char *host;
} carmen_voice_interface_can_line_message;

#define CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_NAME	"carmen_voice_interface_can_line_message"
#define CARMEN_VOICE_INTERFACE_CAN_LINE_MESSAGE_FMT	"{string,double,string}"

#ifdef __cplusplus
}
#endif


#endif /* SRC_VOICE_INTERFACE_VOICE_INTERFACE_MESSAGES_H_ */

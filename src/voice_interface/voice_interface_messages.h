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

#define SET_COURSE 	1
#define SET_SPEED	2
#define SET_GRAPH	3

typedef struct
{
	int command_id;
	char *command;

	double timestamp;
	char *host;
} carmen_voice_interface_command_message;

#define CARMEN_VOICE_INTERFACE_COMMAND_MESSAGE_NAME	"carmen_voice_interface_command_message"
#define CARMEN_VOICE_INTERFACE_COMMAND_MESSAGE_FMT	"{int,string,double,string}"

#ifdef __cplusplus
}
#endif


#endif /* SRC_VOICE_INTERFACE_VOICE_INTERFACE_MESSAGES_H_ */

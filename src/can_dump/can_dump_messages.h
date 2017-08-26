/*
 * can_dump_messages.h
 *
 *  Created on: Aug 24, 2017
 *      Author: alberto
 */

#ifndef SRC_CAN_DUMP_CAN_DUMP_MESSAGES_H_
#define SRC_CAN_DUMP_CAN_DUMP_MESSAGES_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Message Struct Example */
typedef struct
{
	char *can_line;
	double timestamp;
	char *host;
} carmen_can_dump_can_line_message;

#define CARMEN_CAN_DUMP_CAN_LINE_MESSAGE_NAME	"carmen_can_dump_can_line_message"
#define CARMEN_CAN_DUMP_CAN_LINE_MESSAGE_FMT	"{string,double,string}"

#ifdef __cplusplus
}
#endif


#endif /* SRC_CAN_DUMP_CAN_DUMP_MESSAGES_H_ */

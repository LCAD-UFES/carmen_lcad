/*
 * can_dump_interface.h
 *
 *  Created on: Aug 24, 2017
 *      Author: alberto
 */

#ifndef SRC_CAN_DUMP_CAN_DUMP_INTERFACE_H_
#define SRC_CAN_DUMP_CAN_DUMP_INTERFACE_H_

#include <carmen/carmen.h>
#include <carmen/can_dump_messages.h>



#ifdef __cplusplus
extern "C"
{
#endif

void
carmen_can_dump_subscribe_can_line_message(carmen_can_dump_can_line_message *message, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void
carmen_can_dump_unsubscribe_can_line_message(carmen_handler_t handler);

void
carmen_can_dump_publish_can_line_message(carmen_can_dump_can_line_message *message);

void
carmen_can_dump_define_can_line_message();

#ifdef __cplusplus
}
#endif

#endif /* SRC_CAN_DUMP_CAN_DUMP_INTERFACE_H_ */

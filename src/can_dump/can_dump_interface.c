/*
 * can_dump_interface.c
 *
 *  Created on: Aug 24, 2017
 *      Author: alberto
 */

#include <carmen/can_dump_interface.h>

void
carmen_can_dump_subscribe_can_line_message(carmen_can_dump_can_line_message *message, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_CAN_DUMP_CAN_LINE_MESSAGE_NAME, CARMEN_CAN_DUMP_CAN_LINE_MESSAGE_FMT, message,
			sizeof(carmen_can_dump_can_line_message), handler, subscribe_how);
}


void
carmen_can_dump_unsubscribe_can_line_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_CAN_DUMP_CAN_LINE_MESSAGE_NAME, handler);
}


void
carmen_can_dump_publish_can_line_message(carmen_can_dump_can_line_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_CAN_DUMP_CAN_LINE_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_CAN_DUMP_CAN_LINE_MESSAGE_FMT);
}


void
carmen_can_dump_define_can_line_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_CAN_DUMP_CAN_LINE_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_CAN_DUMP_CAN_LINE_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_CAN_DUMP_CAN_LINE_MESSAGE_NAME);
}

#include <carmen/tracker_messages.h>

#ifndef CARMEN_TRACKER_INTERFACE_H
#define CARMEN_TRACKER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_tracker_subscribe_train(
		carmen_tracker_train_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void
carmen_tracker_unsubscribe_train(carmen_handler_t handler);

void
carmen_tracker_subscribe_test(
		carmen_tracker_test_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);
void
carmen_tracker_unsubscribe_test(carmen_handler_t handler);

void
carmen_tracker_subscribe_output(
		carmen_tracker_output_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);
void
carmen_tracker_unsubscribe_output(carmen_handler_t handler);

IPC_RETURN_TYPE
carmen_tracker_define_message_train(void);

IPC_RETURN_TYPE
carmen_tracker_define_message_test(void);

IPC_RETURN_TYPE
carmen_tracker_define_message_output(void);

IPC_RETURN_TYPE
carmen_tracker_define_message_query_test(void);

IPC_RETURN_TYPE
carmen_tracker_define_message_query_train(void);

IPC_RETURN_TYPE
carmen_tracker_define_message_output_train(void);

IPC_RETURN_TYPE
carmen_tracker_define_message_position(void);

void
carmen_tracker_subscribe_query_test(HANDLER_TYPE handler);

void
carmen_tracker_subscribe_query_train(HANDLER_TYPE handler);

carmen_tracker_output_message *
carmen_tracker_query_output_message(
		carmen_tracker_test_message *query,
		double timeout);

carmen_tracker_output_training_message *
carmen_tracker_query_training_message(
		carmen_tracker_train_message *query,
		double timeout);

void
carmen_tracker_subscribe_position_message(carmen_tracker_position_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
position_message_handler(double x, double y, double timestamp);

#ifdef __cplusplus
}
#endif

#endif

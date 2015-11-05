#include <carmen/visual_search_thin_messages.h>

#ifndef CARMEN_VISUAL_SEARCH_THIN_INTERFACE_H
#define CARMEN_VISUAL_SEARCH_THIN_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_visual_search_thin_subscribe_train(
		carmen_visual_search_thin_train_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void
carmen_visual_search_thin_unsubscribe_train(carmen_handler_t handler);

void
carmen_visual_search_thin_subscribe_test(
		carmen_visual_search_thin_test_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);
void
carmen_visual_search_thin_unsubscribe_test(carmen_handler_t handler);

void
carmen_visual_search_thin_subscribe_output(
		carmen_visual_search_thin_output_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);
void
carmen_visual_search_thin_unsubscribe_output(carmen_handler_t handler);

IPC_RETURN_TYPE
carmen_visual_search_thin_define_message_train(void);

IPC_RETURN_TYPE
carmen_visual_search_thin_define_message_test(void);

IPC_RETURN_TYPE
carmen_visual_search_thin_define_message_output(void);

IPC_RETURN_TYPE
carmen_visual_search_thin_define_message_query_test(void);

IPC_RETURN_TYPE
carmen_visual_search_thin_define_message_query_train(void);

IPC_RETURN_TYPE
carmen_visual_search_thin_define_message_output_train(void);

void
carmen_visual_search_thin_subscribe_query_test(HANDLER_TYPE handler);

void
carmen_visual_search_thin_subscribe_query_train(HANDLER_TYPE handler);

carmen_visual_search_thin_output_message *
carmen_visual_search_thin_query_output_message(
		carmen_visual_search_thin_test_message *query,
		double timeout);

carmen_visual_search_thin_output_training_message *
carmen_visual_search_thin_query_training_message(
		carmen_visual_search_thin_train_message *query,
		double timeout);

#ifdef __cplusplus
}
#endif

#endif

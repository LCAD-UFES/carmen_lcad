#include <carmen/visual_search_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_visual_search_subscribe_train(carmen_visual_search_message *visual_search_message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how);

void
carmen_visual_search_unsubscribe_train(carmen_handler_t handler);

void
carmen_visual_search_subscribe_test(carmen_visual_search_test_message *visual_search_message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how);
void
carmen_visual_search_unsubscribe_test(carmen_handler_t handler);

void
carmen_visual_search_subscribe_output(carmen_visual_search_output_message *visual_search_message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how);
void
carmen_visual_search_unsubscribe_output(carmen_handler_t handler);

void
carmen_visual_search_subscribe_state_change(carmen_visual_search_state_change_message *visual_search_state_change_message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how);
			       

IPC_RETURN_TYPE
carmen_visual_search_define_message_train(void);

IPC_RETURN_TYPE
carmen_visual_search_define_message_test(void);

IPC_RETURN_TYPE
carmen_visual_search_define_message_output(void);

IPC_RETURN_TYPE
carmen_visual_search_define_message_state_change(void);

IPC_RETURN_TYPE
carmen_visual_search_define_message_query(void);

void
carmen_visual_search_subscribe_query(HANDLER_TYPE handler);

carmen_visual_search_output_message *
carmen_visual_search_query_output_message(carmen_visual_search_test_message *query, double timeout);

#ifdef __cplusplus
}
#endif

// @}

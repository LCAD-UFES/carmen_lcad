#include <carmen/visual_tracker_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_visual_tracker_subscribe_train(carmen_visual_tracker_train_message *visual_tracker_message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how);

void
carmen_visual_tracker_unsubscribe_train(carmen_handler_t handler);

void
carmen_visual_tracker_subscribe_test(carmen_visual_tracker_test_message *visual_tracker_message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how);
void
carmen_visual_tracker_unsubscribe_test(carmen_handler_t handler);

void
carmen_visual_tracker_subscribe_output(carmen_visual_tracker_output_message *visual_tracker_message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how);
void
carmen_visual_tracker_unsubscribe_output(carmen_handler_t handler);


IPC_RETURN_TYPE
carmen_visual_tracker_define_message_train(void);

IPC_RETURN_TYPE
carmen_visual_tracker_define_message_test(void);

IPC_RETURN_TYPE
carmen_visual_tracker_define_message_output(void);

#ifdef __cplusplus
}
#endif

// @}

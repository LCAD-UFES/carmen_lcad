#include <carmen/landmark_localization_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_landmark_localization_subscribe_state_message(carmen_landmark_localization_state_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_landmark_localization_unsubscribe_state_message(carmen_handler_t handler);

void
carmen_landmark_localization_define_messages();

#ifdef __cplusplus
}
#endif

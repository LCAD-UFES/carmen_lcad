#include <carmen/facial_greeting_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_facial_greeting_subscribe_default_message(carmen_facial_greeting_default_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_facial_greeting_unsubscribe_default_message(carmen_handler_t handler);

void
carmen_facial_greeting_define_default_message();

void
carmen_facial_greeting_subscribe_face_recog_message(carmen_face_recog_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_facial_greeting_unsubscribe_face_recog_message(carmen_handler_t handler);

void
carmen_facial_greeting_define_face_recog_message();

#ifdef __cplusplus
}
#endif


// @}


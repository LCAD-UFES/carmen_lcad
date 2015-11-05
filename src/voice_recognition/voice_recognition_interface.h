#include <carmen/voice_recognition_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void carmen_voice_recognition_subscribe_message(carmen_voice_recognition_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
void carmen_voice_recognition_unsubscribe_message(carmen_handler_t handler);
void carmen_voice_recognition_define_messages();

#ifdef __cplusplus
}
#endif


// @}


#include <carmen/stereo_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

char *
carmen_stereo_message_name(int camera);

void
carmen_stereo_unsubscribe(int camera, carmen_handler_t handler);

IPC_RETURN_TYPE
carmen_stereo_define_messages(int camera);

void
carmen_stereo_subscribe(int camera,
    carmen_simple_stereo_disparity_message *message,
    carmen_handler_t handler, carmen_subscribe_t subscribe_how);

IPC_RETURN_TYPE
carmen_stereo_publish_message(int camera,
    carmen_simple_stereo_disparity_message *message);

#ifdef __cplusplus
}
#endif


// @}


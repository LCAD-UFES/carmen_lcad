#include <carmen/bumblebee_basic_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

IPC_RETURN_TYPE
carmen_bumblebee_basic_define_messages(int camera);

void
carmen_bumblebee_basic_subscribe_stereoimage(int camera,
		carmen_bumblebee_basic_stereoimage_message *message,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_bumblebee_basic_unsubscribe_stereoimage(int camera, carmen_handler_t handler);

IPC_RETURN_TYPE
carmen_bumblebee_basic_publish_message(int camera,
		const carmen_bumblebee_basic_stereoimage_message *message);

#ifdef __cplusplus
}
#endif


// @}


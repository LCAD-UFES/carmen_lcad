#include <carmen/minoru_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_minoru_subscribe_stereoimage(carmen_minoru_stereoimage_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_minoru_unsubscribe_stereoimage(carmen_handler_t handler);

void
carmen_minoru_define_messages();

void
carmen_minoru_define_bumblebee_fake_messages();

IPC_RETURN_TYPE
carmen_minoru_publish_message(
		const carmen_minoru_stereoimage_message *message);

IPC_RETURN_TYPE
carmen_minoru_publish_bumblebee_fake_message(
		const carmen_minoru_stereoimage_message *message);

#ifdef __cplusplus
}
#endif


// @}


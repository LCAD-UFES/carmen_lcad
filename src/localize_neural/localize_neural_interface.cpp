#include <carmen/carmen.h>
#include "localize_neural_messages.h"

void
carmen_localize_neural_subscribe_imagepos_keyframe_message(carmen_localize_neural_imagepos_message
		*imagepos, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char *)CARMEN_LOCALIZE_NEURAL_IMAGEPOS_KEYFRAME_NAME,
			(char *)CARMEN_LOCALIZE_NEURAL_IMAGEPOS_FMT,
			imagepos, sizeof(carmen_localize_neural_imagepos_message),
			handler, subscribe_how);
}

void
carmen_localize_neural_subscribe_imagepos_curframe_message(carmen_localize_neural_imagepos_message
		*imagepos, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char *)CARMEN_LOCALIZE_NEURAL_IMAGEPOS_CURFRAME_NAME,
			(char *)CARMEN_LOCALIZE_NEURAL_IMAGEPOS_FMT,
			imagepos, sizeof(carmen_localize_neural_imagepos_message),
			handler, subscribe_how);
}

void
carmen_localize_neural_unsubscribe_imagepos_keyframe_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char *)CARMEN_LOCALIZE_NEURAL_IMAGEPOS_KEYFRAME_NAME, handler);
}

void
carmen_localize_neural_unsubscribe_imagepos_curframe_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char *)CARMEN_LOCALIZE_NEURAL_IMAGEPOS_CURFRAME_NAME, handler);
}

void
carmen_localize_neural_publish_imagepos_keyframe_message(carmen_localize_neural_imagepos_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_LOCALIZE_NEURAL_IMAGEPOS_KEYFRAME_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LOCALIZE_NEURAL_IMAGEPOS_KEYFRAME_NAME);
}

void
carmen_localize_neural_publish_imagepos_curframe_message(carmen_localize_neural_imagepos_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_LOCALIZE_NEURAL_IMAGEPOS_CURFRAME_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LOCALIZE_NEURAL_IMAGEPOS_CURFRAME_NAME);
}


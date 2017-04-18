#include <carmen/carmen.h>
#include <carmen/semantic_segmentation_interface.h>

void
carmen_semantic_segmentation_subscribe_image_message(carmen_semantic_segmentation_image_message *message,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_SEMANTIC_SEGMENTATION_IMAGE_NAME,
			CARMEN_SEMANTIC_SEGMENTATION_IMAGE_FMT,
			message, sizeof(carmen_semantic_segmentation_image_message),
			handler, subscribe_how);
}


void
carmen_semantic_segmentation_unsubscribe_image_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_SEMANTIC_SEGMENTATION_IMAGE_NAME, handler);
}


void
carmen_semantic_segmentation_define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_SEMANTIC_SEGMENTATION_IMAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_SEMANTIC_SEGMENTATION_IMAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_SEMANTIC_SEGMENTATION_IMAGE_NAME);
}


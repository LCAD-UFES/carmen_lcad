#include <carmen/carmen.h>
#include <carmen/web_cam_interface.h>

void
carmen_web_cam_subscribe_message(carmen_web_cam_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char *) CARMEN_WEB_CAM_MESSAGE_NAME,
			(char *) CARMEN_WEB_CAM_MESSAGE_FMT,
			message, sizeof(carmen_web_cam_message),
			handler, subscribe_how);
}


void
carmen_web_cam_unsubscribe_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char *) CARMEN_WEB_CAM_MESSAGE_NAME, handler);
}


void
carmen_web_cam_define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg((char *) CARMEN_WEB_CAM_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			(char *) CARMEN_WEB_CAM_MESSAGE_FMT);
	carmen_test_ipc_exit(err, (char *) "Could not define", (char *) CARMEN_WEB_CAM_MESSAGE_NAME);
}


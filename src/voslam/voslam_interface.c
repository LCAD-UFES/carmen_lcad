#include <carmen/carmen.h>
#include <carmen/voslam_messages.h>

void
carmen_voslam_subscribe_pointcloud_message(carmen_voslam_pointcloud_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_VOSLAM_POINTCLOUD_MESSAGE_NAME,
			CARMEN_VOSLAM_POINTCLOUD_MESSAGE_FMT,
			message, sizeof(carmen_voslam_pointcloud_message),
			handler, subscribe_how);
}


void
carmen_voslam_unsubscribe_pointcloud_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_VOSLAM_POINTCLOUD_MESSAGE_NAME, handler);
}


void
carmen_voslam_define_messages()
{
	IPC_RETURN_TYPE err;

	// define a mensagem de globalpos
	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

	// define a mensagem do voslam
	err = IPC_defineMsg(CARMEN_VOSLAM_POINTCLOUD_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_VOSLAM_POINTCLOUD_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VOSLAM_POINTCLOUD_MESSAGE_NAME);
}


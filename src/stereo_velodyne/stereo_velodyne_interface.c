#include <carmen/carmen.h>
#include "stereo_velodyne_messages.h"

char *
carmen_stereo_velodyne_scan_message_name(int camera)
{
	char *message_name = (char*)malloc(128 * sizeof(char));
	sprintf(message_name, "%s%d", CARMEN_STEREO_VELODYNE_SCAN_MESSAGE_NAME, camera);
	return message_name;
}

void
carmen_stereo_velodyne_unsubscribe_scan_message(int camera, carmen_handler_t handler)
{
	char *message_name = carmen_stereo_velodyne_scan_message_name(camera);
	carmen_unsubscribe_message(message_name, handler);
	free(message_name);
}

IPC_RETURN_TYPE
carmen_stereo_velodyne_define_messages(int camera)
{
	IPC_RETURN_TYPE err;

	char *message_name = carmen_stereo_velodyne_scan_message_name(camera);
	err = IPC_defineMsg(message_name, IPC_VARIABLE_LENGTH, CARMEN_STEREO_VELODYNE_SCAN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", message_name);
	free(message_name);
	return err;
}

void
carmen_stereo_velodyne_subscribe_scan_message(int camera,
		carmen_velodyne_variable_scan_message *message,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	char *message_name = carmen_stereo_velodyne_scan_message_name(camera);
	carmen_subscribe_message(message_name, CARMEN_STEREO_VELODYNE_SCAN_MESSAGE_FMT,
			message, sizeof(carmen_velodyne_variable_scan_message),
			handler, subscribe_how);
	free(message_name);
	printf("\nSubscribed to Stereo Messages (From Camera %d)!\n", camera);
}


IPC_RETURN_TYPE
carmen_stereo_velodyne_publish_message(int camera,
		carmen_velodyne_variable_scan_message *message)
{
	IPC_RETURN_TYPE err;

	char *message_name = carmen_stereo_velodyne_scan_message_name(camera);

	err = IPC_publishData(message_name, message);
	carmen_test_ipc_exit(err, "Could not publish", message_name);
	free(message_name);

	return err;
}


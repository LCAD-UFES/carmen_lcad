#include <carmen/carmen.h>
#include <carmen/polar_slam_messages.h>

void
carmen_polar_slam_subscribe_message(carmen_polar_slam_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_POLAR_SLAM_MESSAGE_NAME,
			CARMEN_POLAR_SLAM_MESSAGE_FMT,
			message, sizeof(carmen_polar_slam_message),
			handler, subscribe_how);
}


void
carmen_polar_slam_unsubscribe_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_POLAR_SLAM_MESSAGE_NAME, handler);
}


void
carmen_polar_slam_define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_POLAR_SLAM_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_POLAR_SLAM_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_POLAR_SLAM_MESSAGE_NAME);
}


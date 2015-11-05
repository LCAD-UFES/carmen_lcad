#include <carmen/carmen.h>
#include <carmen/laslam_messages.h>

void
carmen_laslam_define_landmark_message()
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_LASLAM_LANDMARK_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LASLAM_LANDMARK_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LASLAM_LANDMARK_MESSAGE_NAME);
}

void
carmen_laslam_subscribe_landmark_message(
		carmen_laslam_landmark_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_LASLAM_LANDMARK_MESSAGE_NAME, CARMEN_LASLAM_LANDMARK_MESSAGE_FMT,
			message, sizeof(carmen_laslam_landmark_message), handler, subscribe_how);
}

void
carmen_laslam_unsubscribe_landmark_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_LASLAM_LANDMARK_MESSAGE_NAME, handler);
}

void
carmen_laslam_publish_landmark_message(carmen_laslam_landmark_message* message)
{
	IPC_RETURN_TYPE err;
	err = IPC_publishData(CARMEN_LASLAM_LANDMARK_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LASLAM_LANDMARK_MESSAGE_FMT);
}

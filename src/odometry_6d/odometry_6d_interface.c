#include <carmen/carmen.h>
#include <carmen/odometry_6d_messages.h>

void
carmen_odometry_6d_subscribe_message(carmen_odometry_6d_message *message, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ODOMETRY_6D_NAME, CARMEN_ODOMETRY_6D_FMT, message,
			sizeof(carmen_odometry_6d_message), handler, subscribe_how);
}


void
carmen_odometry_6d_unsubscribe_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_ODOMETRY_6D_NAME, handler);
}


void
carmen_odometry_6d_define_message(void)
{
	IPC_RETURN_TYPE err = IPC_defineMsg(CARMEN_ODOMETRY_6D_NAME, IPC_VARIABLE_LENGTH, CARMEN_ODOMETRY_6D_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ODOMETRY_6D_NAME);
}


void
carmen_odometry_6d_publish_message(carmen_odometry_6d_message *message)
{
	static int defined = 0;
	if (!defined)
	{
		carmen_odometry_6d_define_message();
		defined = 1;
	}
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_ODOMETRY_6D_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_ODOMETRY_6D_NAME);
}

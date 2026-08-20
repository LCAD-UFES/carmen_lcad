#include <carmen/carmen.h>
#include <carmen/pi_nit_interface.h>


IPC_RETURN_TYPE
pi_nit_define_status_message(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_PI_NIT_STATUS_NAME, IPC_VARIABLE_LENGTH, CARMEN_PI_NIT_STATUS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_PI_NIT_STATUS_NAME);

	return (err);
}


void
pi_nit_subscribe_status_message(carmen_pi_nit_status_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char *) CARMEN_PI_NIT_STATUS_NAME, (char *) CARMEN_PI_NIT_STATUS_FMT, message,
			sizeof(carmen_pi_nit_status_message), handler, subscribe_how);
}


void
pi_nit_unsubscribe_status_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char *) CARMEN_PI_NIT_STATUS_NAME, handler);
}


IPC_RETURN_TYPE
pi_nit_publish_status_message(carmen_pi_nit_status_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_PI_NIT_STATUS_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_PI_NIT_STATUS_NAME);

	return (err);
}

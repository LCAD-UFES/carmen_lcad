#include <carmen/carmen.h>
#include <carmen/xsens_mtig_messages.h>

void
carmen_xsens_mtig_subscribe_message(carmen_xsens_mtig_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(	CARMEN_XSENS_MTIG_NAME,
		  	  	  	  	  	CARMEN_XSENS_MTIG_FMT,
		  	  	  	  	  	message, sizeof(carmen_xsens_mtig_message),
		  	  	  	  	  	handler, subscribe_how);
}


void
carmen_xsens_mtig_unsubscribe_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_XSENS_MTIG_NAME, handler);
}


void
carmen_xsens_mtig_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_XSENS_MTIG_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_MTIG_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_MTIG_NAME);

  err = IPC_defineMsg(CARMEN_XSENS_MTIG_RAW_GPS_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_MTIG_RAW_GPS_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_MTIG_RAW_GPS_NAME);
}

void
publish_mti_quat_message(carmen_xsens_global_quat_message message)
{
   IPC_RETURN_TYPE err;

   err = IPC_publishData(CARMEN_XSENS_GLOBAL_QUAT_NAME, &message);
   carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_GLOBAL_QUAT_FMT);
}

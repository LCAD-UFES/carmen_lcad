#include <carmen/carmen.h>
#include <carmen/ekf_odometry_messages.h>

void
carmen_ekf_odometry_subscribe_odometry_message(carmen_ekf_odometry_odometry_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_EKF_ODOMETRY_ODOMETRY_MESSAGE_NAME,
		  	  	  	  	   CARMEN_EKF_ODOMETRY_ODOMETRY_MESSAGE_FMT,
                           message, sizeof(carmen_ekf_odometry_odometry_message),
                           handler, subscribe_how);
}


void
carmen_ekf_odometry_unsubscribe_odometry_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_EKF_ODOMETRY_ODOMETRY_MESSAGE_NAME, handler);
}


void
carmen_ekf_odometry_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_EKF_ODOMETRY_ODOMETRY_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
		  CARMEN_EKF_ODOMETRY_ODOMETRY_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_EKF_ODOMETRY_ODOMETRY_MESSAGE_NAME);
}


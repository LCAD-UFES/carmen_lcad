#include <carmen/carmen.h>


void
carmen_imu_subscribe_imu_message(carmen_imu_message* msg,
				 carmen_handler_t handler,
				 carmen_subscribe_t subscribe_how) {

  carmen_subscribe_message(CARMEN_IMU_MESSAGE_NAME,
			   CARMEN_IMU_MESSAGE_FMT,
			   msg, sizeof(carmen_imu_message), handler,
			   subscribe_how);
}

void
carmen_imu_unsubscribe_imu_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_IMU_MESSAGE_NAME,  handler);
}


void
carmen_imu_define_imu_message()
{
  IPC_RETURN_TYPE err;
  
  err = IPC_defineMsg(CARMEN_IMU_MESSAGE_NAME,IPC_VARIABLE_LENGTH, 
		      CARMEN_IMU_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_IMU_MESSAGE_NAME);
}



void
carmen_imu_publish_imu_message(carmen_imu_message* msg)  {

  if (!msg)
    return;

  IPC_RETURN_TYPE err;
  err = IPC_publishData(CARMEN_IMU_MESSAGE_NAME, msg);
  carmen_test_ipc_exit(err, "Could not publish", CARMEN_IMU_MESSAGE_NAME);
}

void
carmen_imu_subscribe_alive_message(carmen_imu_alive_message *alive,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_IMU_ALIVE_NAME, 
			   CARMEN_IMU_ALIVE_FMT,
			   alive, sizeof(carmen_imu_alive_message), handler,
			   subscribe_how);
}

void
carmen_imu_unsubscribe_alive_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_IMU_ALIVE_NAME, handler);
}

void
carmen_imu_define_alive_message()
{
  carmen_ipc_define_test_exit(CARMEN_IMU_ALIVE_NAME, 
			      CARMEN_IMU_ALIVE_FMT);
}


void
carmen_imu_publish_alive_message(int value)  {

  carmen_imu_alive_message msg;
/*   msg.timestamp = carmen_get_time(); */
/*   msg.host = carmen_get_host(); */
  msg.isalive = value;

  IPC_RETURN_TYPE err;
  err = IPC_publishData(CARMEN_IMU_ALIVE_NAME, &msg);
  carmen_test_ipc_exit(err, "Could not publish", CARMEN_IMU_ALIVE_NAME);
}


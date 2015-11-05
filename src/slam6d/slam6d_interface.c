#include <carmen/carmen.h>
#include <carmen/slam6d_messages.h>

void
carmen_slam6d_subscribe_pointcloud_message(carmen_slam6d_pointcloud_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_SLAM6D_POINTCLOUD_MESSAGE_NAME,
		  	  	  	  	   CARMEN_SLAM6D_POINTCLOUD_MESSAGE_FMT,
                           message, sizeof(carmen_slam6d_pointcloud_message),
                           handler, subscribe_how);
}


void
carmen_slam6d_unsubscribe_pointcloud_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_SLAM6D_POINTCLOUD_MESSAGE_NAME, handler);
}


void
carmen_slam6d_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_SLAM6D_POINTCLOUD_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
		  CARMEN_SLAM6D_POINTCLOUD_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_SLAM6D_POINTCLOUD_MESSAGE_NAME);
}


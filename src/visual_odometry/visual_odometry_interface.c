#include <carmen/carmen.h>
#include <carmen/visual_odometry_messages.h>

void
carmen_visual_odometry_subscribe_pose6d_message(carmen_visual_odometry_pose6d_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_NAME,
		  	  	  	  	   CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_FMT,
                           message, sizeof(carmen_visual_odometry_pose6d_message),
                           handler, subscribe_how);
}


void
carmen_visual_odometry_unsubscribe_pose6d_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_NAME, handler);
}


void
carmen_visual_odometry_subscribe_image_message(carmen_visual_odometry_image_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_VISUAL_ODOMETRY_IMAGE_MESSAGE_NAME,
		  	  	  	  	   CARMEN_VISUAL_ODOMETRY_IMAGE_MESSAGE_FMT,
                           message, sizeof(carmen_visual_odometry_image_message),
                           handler, subscribe_how);
}

void
carmen_visual_odometry_unsubscribe_image_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_VISUAL_ODOMETRY_IMAGE_MESSAGE_NAME, handler);
}

void
carmen_visual_odometry_define_messages()
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_VISUAL_ODOMETRY_IMAGE_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_VISUAL_ODOMETRY_IMAGE_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_ODOMETRY_IMAGE_MESSAGE_NAME);
}


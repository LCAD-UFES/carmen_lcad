#include <carmen/carmen.h>
#include <carmen/xsens_messages.h>

void
carmen_xsens_define_messages()
{
    IPC_RETURN_TYPE err;

    /* register xsens's global message */
    err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_QUAT_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_QUAT_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_QUAT_NAME);

    err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_EULER_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_EULER_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_EULER_NAME);

    err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_MATRIX_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_MATRIX_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_MATRIX_NAME);
}

void
carmen_xsens_subscribe_xsens_global_matrix_message(carmen_xsens_global_matrix_message 
					    *xsens_global,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_XSENS_GLOBAL_MATRIX_NAME,
                             CARMEN_XSENS_GLOBAL_MATRIX_FMT,
                             xsens_global, sizeof(carmen_xsens_global_matrix_message),
		                     handler, subscribe_how);

}


void
carmen_xsens_subscribe_xsens_global_euler_message(carmen_xsens_global_euler_message 
					    *xsens_global,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_XSENS_GLOBAL_EULER_NAME, 
                             CARMEN_XSENS_GLOBAL_EULER_FMT,
                             xsens_global, sizeof(carmen_xsens_global_euler_message), 
		                     handler, subscribe_how);

}


void
carmen_xsens_subscribe_xsens_global_quat_message(carmen_xsens_global_quat_message 
					    *xsens_global,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_XSENS_GLOBAL_QUAT_NAME, 
                             CARMEN_XSENS_GLOBAL_QUAT_FMT,
                             xsens_global, sizeof(carmen_xsens_global_quat_message), 
            			     handler, subscribe_how);

}

// =========== 

void
carmen_xsens_unsubscribe_xsens_global_matrix_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_XSENS_GLOBAL_MATRIX_NAME, handler);
}


void
carmen_xsens_unsubscribe_xsens_global_euler_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_XSENS_GLOBAL_EULER_NAME, handler);
}


void
carmen_xsens_unsubscribe_xsens_global_quat_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_XSENS_GLOBAL_QUAT_NAME, handler);
}

void
carmen_xsens_subscribe_xsens_global_message(carmen_xsens_global_message
					    *xsens_global,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_XSENS_GLOBAL_NAME,
                           CARMEN_XSENS_GLOBAL_FMT,
                           xsens_global, sizeof(carmen_xsens_global_message),
			   handler, subscribe_how);
  fprintf(stderr, "\nSubscribe xsens!\n");
  printf("\nSubscribe xsens!\n");
}

void
carmen_xsens_unsubscribe_xsens_global_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_XSENS_GLOBAL_NAME, handler);
}


carmen_orientation_3D_t xsens_get_pose_from_message(carmen_xsens_global_matrix_message message)
{
  carmen_orientation_3D_t orientation;

  orientation.roll = atan2(message.matrix_data.m_data[2][1], message.matrix_data.m_data[2][2]);
  orientation.pitch = -asin(message.matrix_data.m_data[2][0]);
  orientation.yaw = atan2(message.matrix_data.m_data[1][0], message.matrix_data.m_data[0][0]);

  return orientation;
}

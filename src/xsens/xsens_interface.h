#ifndef CARMEN_XSENS_INTERFACE_H
#define CARMEN_XSENS_INTERFACE_H

#include "xsens_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_xsens_define_messages();

void
carmen_xsens_subscribe_xsens_global_message(carmen_xsens_global_message 
					    *xsens_global,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how);

void carmen_xsens_subscribe_xsens_global_matrix_message(carmen_xsens_global_matrix_message 
					    *xsens_global,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how);

void carmen_xsens_subscribe_xsens_global_euler_message(carmen_xsens_global_euler_message 
					    *xsens_global,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how);
					    
void carmen_xsens_subscribe_xsens_global_quat_message(carmen_xsens_global_quat_message 
					    *xsens_global,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how);

/*
void
carmen_xsens_unsubscribe_xsens_global_message(carmen_handler_t handler);
*/

void
carmen_xsens_unsubscribe_xsens_global_matrix_message(carmen_handler_t handler);

void
carmen_xsens_unsubscribe_xsens_global_euler_message(carmen_handler_t handler);

void
carmen_xsens_unsubscribe_xsens_global_quat_message(carmen_handler_t handler);

carmen_orientation_3D_t
xsens_get_pose_from_message(carmen_xsens_global_matrix_message message);

#ifdef __cplusplus
}
#endif

#endif
// @}

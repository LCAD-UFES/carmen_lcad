#ifndef CARMEN_ODOMETRY_6D_INTERFACE_H
#define CARMEN_ODOMETRY_6D_INTERFACE_H

#include <carmen/carmen.h>
#include <carmen/odometry_6d_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void carmen_odometry_6d_subscribe_message(carmen_odometry_6d_message *message, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void carmen_odometry_6d_unsubscribe_message(carmen_handler_t handler);

void carmen_odometry_6d_define_message(void);

void carmen_odometry_6d_publish_message(carmen_odometry_6d_message *message);

#ifdef __cplusplus
}
#endif

#endif

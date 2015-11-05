#ifndef CARMEN_SLAM6D_INTERFACE_H
#define CARMEN_SLAM6D_INTERFACE_H

#include "slam6d_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_slam6d_subscribe_pointcloud_message(carmen_slam6d_pointcloud_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_slam6d_unsubscribe_pointcloud_message(carmen_handler_t handler);

void
carmen_slam6d_define_messages();

#ifdef __cplusplus
}
#endif

#endif

// @}


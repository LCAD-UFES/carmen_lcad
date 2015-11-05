#ifndef CARMEN_VOSLAM_INTERFACE_H
#define CARMEN_VOSLAM_INTERFACE_H

#include "voslam_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_voslam_subscribe_pointcloud_message(carmen_voslam_pointcloud_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_voslam_unsubscribe_pointcloud_message(carmen_handler_t handler);

void
carmen_voslam_define_messages();

#ifdef __cplusplus
}
#endif

#endif

// @}


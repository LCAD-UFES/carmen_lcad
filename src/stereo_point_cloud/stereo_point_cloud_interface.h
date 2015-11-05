 
#ifndef CARMEN_STEREO_POINT_CLOUD_INTERFACE_H
#define CARMEN_STEREO_POINT_CLOUD_INTERFACE_H

#include "stereo_point_cloud_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_stereo_point_cloud_subscribe_stereo_point_cloud_message(	carmen_stereo_point_cloud_message *stereo_point_cloud_message,
																carmen_handler_t handler,
																carmen_subscribe_t subscribe_how);

void
carmen_stereo_point_cloud_unsubscribe_stereo_point_cloud_message(carmen_handler_t handler);


#ifdef __cplusplus
}
#endif

#endif

#ifndef VIEWER_3D_INTERFACE_H_
#define VIEWER_3D_INTERFACE_H_

#include "viewer_3D_messages.h"

#ifdef __cplusplus
extern "C"
{
#endif

void carmen_viewer_3D_define_map_view_message();

void carmen_viewer_3D_subscribe_map_view_message(carmen_viewer_3D_map_view_message *msg,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void carmen_viewer_3D_unsubscribe_map_view_message(carmen_handler_t handler);

void carmen_viewer_3D_publish_map_view_message(int width, int height, int image_size, unsigned char *raw_image,
		carmen_pose_3D_t camera_pose, carmen_pose_3D_t camera_offset);


#ifdef __cplusplus
}
#endif

#endif

// @}


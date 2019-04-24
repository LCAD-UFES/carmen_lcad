#ifndef CARMEN_CAMERA_INTERFACE_H
#define CARMEN_CAMERA_INTERFACE_H

#include <carmen/camera_messages.h>

#ifdef __cplusplus
extern "C" {
#endif


// Subscribe to images from camera_xxxxcam (e.g., camera_quickcam)

void
carmen_camera_subscribe_images(carmen_camera_image_message *image, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

IPC_RETURN_TYPE
carmen_camera_publish_message(carmen_camera_image_message *msg);

#ifdef __cplusplus
}
#endif

#endif

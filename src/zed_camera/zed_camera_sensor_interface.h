#include "zed_camera_sensor_messages.h"

#ifdef __cplusplus
extern "C" 
{
#endif

IPC_RETURN_TYPE
carmen_zed_camera_sensor_publish_stereoimage_message(const carmen_zed_camera_sensor_stereoimage_message *message);

IPC_RETURN_TYPE
carmen_zed_camera_sensor_publish_depthmap_message(const carmen_zed_camera_sensor_depthmap_message *message);

void
carmen_zed_camera_sensor_subscribe_stereoimage(carmen_zed_camera_sensor_stereoimage_message *message,
                                               carmen_handler_t handler,
                                               carmen_subscribe_t subscribe_how);

void
carmen_zed_camera_sensor_subscribe_depthmap(carmen_zed_camera_sensor_depthmap_message *message,
                                            carmen_handler_t handler,
                                            carmen_subscribe_t subscribe_how);

void
carmen_zed_camera_sensor_unsubscribe_stereoimage(carmen_handler_t handler);

void
carmen_zed_camera_sensor_unsubscribe_depthmap(carmen_handler_t handler);

IPC_RETURN_TYPE
carmen_zed_camera_sensor_define_stereoimage_messages();

IPC_RETURN_TYPE
carmen_zed_camera_sensor_define_depthmap_messages();

#ifdef __cplusplus
}
#endif


// @}


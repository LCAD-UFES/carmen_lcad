#ifndef XYZ_POINTCLOUD_LIDAR_INTERFACE_H_
#define XYZ_POINTCLOUD_LIDAR_INTERFACE_H_

#include "carmen/global.h"
#include "carmen/ipc_wrapper.h"
#include "xyz_pointcloud_lidar_messages.h"

#ifdef __cplusplus
extern "C"
{
#endif

void
carmen_xyz_pointcloud_lidar_create_variable_xyz_pointcloud_message_name(int sensor_id, char message_name[]);

IPC_RETURN_TYPE
carmen_xyz_pointcloud_lidar_publish_xyz_pointcloud_message(carmen_xyz_pointcloud_lidar_message *message, int sensor_id);

void
carmen_subscribe_xyz_pointcloud_message(carmen_xyz_pointcloud_lidar_message *message,
										 carmen_handler_t handler,
										 carmen_subscribe_t subscribe_how,
										 int sensor_id);

void
carmen_unsubscribe_xyz_pointcloud_message(carmen_handler_t handler);

void
load_xyz_lidar_config(int argc, char** argv, int xyz_lidar_id, carmen_xyz_lidar_config **xyz_lidar_config);

#ifdef __cplusplus
}
#endif

#endif

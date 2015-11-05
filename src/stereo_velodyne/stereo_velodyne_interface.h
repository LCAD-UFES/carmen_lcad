#include <carmen/carmen.h>
#include <carmen/velodyne_messages.h>

#ifndef CARMEN_STEREO_VELODYNE_INTERFACE_H
#define CARMEN_STEREO_VELODYNE_INTERFACE_H

#ifdef __cplusplus
extern "C"
{
#endif

char *
carmen_stereo_velodyne_scan_message_name(int camera);

void
carmen_stereo_velodyne_unsubscribe_scan_message(int camera, carmen_handler_t handler);


IPC_RETURN_TYPE
carmen_stereo_velodyne_define_messages(int camera);


void
carmen_stereo_velodyne_subscribe_scan_message(int camera,
	carmen_velodyne_variable_scan_message *message,
    carmen_handler_t handler, carmen_subscribe_t subscribe_how);


IPC_RETURN_TYPE
carmen_stereo_velodyne_publish_message(int camera,
		carmen_velodyne_variable_scan_message *message);


#ifdef __cplusplus
}
#endif

#endif
// @}


 
#ifndef CARMEN_VELODYNE_ICP_INTERFACE_H
#define CARMEN_VELODYNE_ICP_INTERFACE_H

#include "velodyne_icp_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_velodyne_icp_subscribe_velodyne_icp_message(	carmen_velodyne_icp_message *velodyne_icp_message,
					carmen_handler_t handler,
					carmen_subscribe_t subscribe_how);

void
carmen_velodyne_icp_unsubscribe_velodyne_icp_message(carmen_handler_t handler);


#ifdef __cplusplus
}
#endif

#endif

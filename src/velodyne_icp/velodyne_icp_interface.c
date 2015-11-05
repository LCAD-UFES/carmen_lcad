#include <carmen/carmen.h>
#include <carmen/velodyne_icp_messages.h>

void
carmen_velodyne_icp_subscribe_velodyne_icp_message(	carmen_velodyne_icp_message *velodyne_icp_message,
					carmen_handler_t handler,
					carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_VELODYNE_ICP_NAME,
                           CARMEN_VELODYNE_ICP_FMT,
                           velodyne_icp_message, sizeof(carmen_velodyne_icp_message),
			   handler, subscribe_how);
}

void
carmen_velodyne_icp_unsubscribe_velodyne_icp_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_VELODYNE_ICP_NAME, handler);
}


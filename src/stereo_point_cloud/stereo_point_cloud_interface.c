#include <carmen/carmen.h>
#include <carmen/stereo_point_cloud_messages.h>

void
carmen_stereo_point_cloud_subscribe_stereo_point_cloud_message(	carmen_stereo_point_cloud_message *stereo_point_cloud_message,
					carmen_handler_t handler,
					carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(	CARMEN_STEREO_POINT_CLOUD_NAME, 
                           	CARMEN_STEREO_POINT_CLOUD_FMT,
                           	stereo_point_cloud_message, sizeof(carmen_stereo_point_cloud_message), 
			   				handler, subscribe_how);
}

void
carmen_stereo_point_cloud_unsubscribe_stereo_point_cloud_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_STEREO_POINT_CLOUD_NAME, handler);
}


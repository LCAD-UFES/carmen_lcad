
#ifndef CARMEN_WEB_CAM_INTERFACE_H
#define CARMEN_WEB_CAM_INTERFACE_H

#include <carmen/web_cam_messages.h>

#ifdef __cplusplus
extern "C"
{
#endif

	void
	carmen_web_cam_subscribe_message(carmen_web_cam_message *message,
			carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	carmen_web_cam_unsubscribe_message(carmen_handler_t handler);

	void
	carmen_web_cam_define_messages();

#ifdef __cplusplus
}
#endif

#endif

// @}


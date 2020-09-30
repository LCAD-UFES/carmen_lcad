#include <carmen/carmen.h>
#include "viewer_3D_interface.h"


void
carmen_viewer_3D_define_map_view_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_VIEWER_3D_MAP_VIEW_NAME, IPC_VARIABLE_LENGTH, CARMEN_VIEWER_3D_MAP_VIEW_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_VIEWER_3D_MAP_VIEW_NAME);
}


void
carmen_viewer_3D_subscribe_map_view_message(carmen_viewer_3D_map_view_message *msg,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_VIEWER_3D_MAP_VIEW_NAME, CARMEN_VIEWER_3D_MAP_VIEW_FMT,
			msg, sizeof(carmen_viewer_3D_map_view_message), handler, subscribe_how);
}


void
carmen_viewer_3D_unsubscribe_map_view_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_VIEWER_3D_MAP_VIEW_NAME, handler);
}


void
carmen_viewer_3D_publish_map_view_message(int width, int height, int image_size, unsigned char *raw_image,
		carmen_pose_3D_t camera_pose, carmen_pose_3D_t camera_offset)
{
	IPC_RETURN_TYPE err;
	carmen_viewer_3D_map_view_message msg;

	static int firstime = 1;

	if (firstime)
	{
		carmen_viewer_3D_define_map_view_message();
		firstime = 0;
	}

	msg.width = width;
	msg.height = height;
	msg.image_size = image_size;
	msg.raw_image = raw_image;
	msg.camera_pose = camera_pose;
	msg.camera_offset = camera_offset;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_VIEWER_3D_MAP_VIEW_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_VIEWER_3D_MAP_VIEW_NAME);
}

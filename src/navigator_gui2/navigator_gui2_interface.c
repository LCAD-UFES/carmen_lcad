#include <carmen/carmen.h>
#include "navigator_gui2_interface.h"


void
carmen_navigator_gui_define_path_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_NAVIGATOR_GUI_PATH_NAME, IPC_VARIABLE_LENGTH, CARMEN_NAVIGATOR_GUI_PATH_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_NAVIGATOR_GUI_PATH_NAME);
}


void
carmen_navigator_gui_subscribe_path_message(carmen_navigator_gui_path_message *msg,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_NAVIGATOR_GUI_PATH_NAME, CARMEN_NAVIGATOR_GUI_PATH_FMT,
			msg, sizeof(carmen_navigator_gui_path_message), handler, subscribe_how);
}


void
carmen_navigator_gui_unsubscribe_path_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_NAVIGATOR_GUI_PATH_NAME, handler);
}


void
carmen_navigator_gui_publish_path_message(carmen_navigator_gui_path_message *msg)
{
	IPC_RETURN_TYPE err;

	static int firstime = 1;

	if (firstime)
	{
		carmen_navigator_gui_define_path_message();
		firstime = 0;
	}

	err = IPC_publishData(CARMEN_NAVIGATOR_GUI_PATH_NAME, msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_GUI_PATH_NAME);
}


void
carmen_navigator_gui_define_map_view_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_NAVIGATOR_GUI_MAP_VIEW_NAME, IPC_VARIABLE_LENGTH, CARMEN_NAVIGATOR_GUI_MAP_VIEW_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_NAVIGATOR_GUI_MAP_VIEW_NAME);
}


void
carmen_navigator_gui_subscribe_map_view_message(carmen_navigator_gui_map_view_message *msg,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_NAVIGATOR_GUI_MAP_VIEW_NAME, CARMEN_NAVIGATOR_GUI_MAP_VIEW_FMT,
			msg, sizeof(carmen_navigator_gui_map_view_message), handler, subscribe_how);
}


void
carmen_navigator_gui_unsubscribe_map_view_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_NAVIGATOR_GUI_MAP_VIEW_NAME, handler);
}


void
carmen_navigator_gui_publish_map_view_message(int width, int height, int image_size, unsigned char *raw_image,
		double x_origin, double y_origin, double resolution)
{
	IPC_RETURN_TYPE err;
	carmen_navigator_gui_map_view_message msg;

	static int firstime = 1;

	if (firstime)
	{
		carmen_navigator_gui_define_map_view_message();
		firstime = 0;
	}

	msg.width = width;
	msg.height = height;
	msg.image_size = image_size;
//	msg.image_size = width * height * 3; /* width*height*bytes_per_pixel */
	msg.raw_image = raw_image;
	msg.x_origin = x_origin;
	msg.y_origin = y_origin;
	msg.resolution = resolution;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

//	printf("Here %d %d %d %d\n", __LINE__, msg.width, msg.height, msg.image_size);
//	printf("Here %d %f %f %f\n", __LINE__, msg.x_origin, msg.y_origin, msg.resolution);


	err = IPC_publishData(CARMEN_NAVIGATOR_GUI_MAP_VIEW_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_GUI_MAP_VIEW_NAME);
}

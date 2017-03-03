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
carmen_navigator_gui_publish_path_message(carmen_navigator_gui_path_message *msg)
{
	static int firstime = 1;

	IPC_RETURN_TYPE err;

	if (firstime)
	{
		carmen_navigator_gui_define_path_message();
		firstime = 0;
	}

	err = IPC_publishData(CARMEN_NAVIGATOR_GUI_PATH_NAME, msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_GUI_PATH_NAME);
}


void
carmen_navigator_gui_subscribe_path_message(carmen_navigator_gui_path_message *msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_NAVIGATOR_GUI_PATH_NAME, CARMEN_NAVIGATOR_GUI_PATH_FMT,
			msg, sizeof(carmen_navigator_gui_path_message), handler, subscribe_how);
}

#include <carmen/carmen.h>
#include <carmen/download_map_streetview_messages.h>

void
carmen_download_map_streetview_subscribe_message(carmen_download_map_streetview_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_DOWNLOAD_MAP_STREETVIEW_MESSAGE_NAME,
			CARMEN_DOWNLOAD_MAP_STREETVIEW_MESSAGE_FMT,
			message, sizeof(carmen_download_map_streetview_message),
			handler, subscribe_how);
}


void
carmen_download_map_streetview_unsubscribe_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_DOWNLOAD_MAP_STREETVIEW_MESSAGE_NAME, handler);
}


void
carmen_download_map_streetview_define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_DOWNLOAD_MAP_STREETVIEW_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_DOWNLOAD_MAP_STREETVIEW_MESSAGE_FMT);

	carmen_test_ipc_exit(err, "Could not define", CARMEN_DOWNLOAD_MAP_STREETVIEW_MESSAGE_NAME);
}


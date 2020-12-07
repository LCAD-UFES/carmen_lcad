#include <carmen/carmen.h>
#include <carmen/download_map_messages.h>

void
carmen_download_map_subscribe_message(carmen_download_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char *) CARMEN_DOWNLOAD_MAP_MESSAGE_NAME,
			(char *) CARMEN_DOWNLOAD_MAP_MESSAGE_FMT,
			message, sizeof(carmen_download_map_message),
			handler, subscribe_how);
}


void
carmen_download_map_unsubscribe_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char *) CARMEN_DOWNLOAD_MAP_MESSAGE_NAME, handler);
}


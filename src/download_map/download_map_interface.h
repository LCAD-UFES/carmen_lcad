
#ifndef CARMEN_DOWNLOAD_MAP_INTERFACE
#define CARMEN_DOWNLOAD_MAP_INTERFACE

#include <carmen/carmen.h>
#include <carmen/download_map_messages.h>

	// Tiramos os ifdef para funcionar

	void carmen_download_map_subscribe_message(carmen_download_map_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void carmen_download_map_unsubscribe_message(carmen_handler_t handler);


#endif

// @}


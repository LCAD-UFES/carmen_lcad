
#ifndef CARMEN_DOWNLOAD_MAP_STREETVIEW_INTERFACE
#define CARMEN_DOWNLOAD_MAP_STREETVIEW_INTERFACE

#include <carmen/download_map_streetview_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

	void carmen_download_map_streetview_subscribe_message(carmen_download_map_streetview_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
	void carmen_download_map_streetview_unsubscribe_message(carmen_handler_t handler);
	void carmen_download_map_streetview_define_messages();

#ifdef __cplusplus
}
#endif

#endif

// @}


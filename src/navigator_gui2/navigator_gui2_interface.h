#ifndef NAVIGATOR_GUI2_INTERFACE_H_
#define NAVIGATOR_GUI2_INTERFACE_H_

#include "navigator_gui2_messages.h"

#ifdef __cplusplus
extern "C"
{
#endif

void carmen_navigator_gui_define_path_message();

void carmen_navigator_gui_subscribe_path_message(carmen_navigator_gui_path_message *msg,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void carmen_navigator_gui_unsubscribe_path_message(carmen_handler_t handler);

void carmen_navigator_gui_publish_path_message(carmen_navigator_gui_path_message *msg);

void carmen_navigator_gui_define_map_view_message();

void carmen_navigator_gui_subscribe_map_view_message(carmen_navigator_gui_map_view_message *msg,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void carmen_navigator_gui_unsubscribe_map_view_message(carmen_handler_t handler);

void carmen_navigator_gui_publish_map_view_message(int width, int height, int image_size, unsigned char *raw_image,
		double x_origin, double y_origin, double resolution);


#ifdef __cplusplus
}
#endif

#endif

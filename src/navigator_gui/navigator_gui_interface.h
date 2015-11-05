/*
 * navigator_gui_interface.h
 *
 *  Created on: 29/01/2013
 *      Author: romulo
 */

#ifndef NAVIGATOR_GUI_INTERFACE_H_
#define NAVIGATOR_GUI_INTERFACE_H_

#include "navigator_gui_messages.h"

#ifdef __cplusplus
extern "C" {
#endif
void carmen_navigator_gui_define_path_message();

void carmen_navigator_gui_publish_path_message(carmen_navigator_gui_path_message *msg);

void carmen_navigator_gui_subscribe_path_message(carmen_navigator_gui_path_message *msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

#ifdef __cplusplus
}
#endif

#endif /* NAVIGATOR_GUI_INTERFACE_H_ */

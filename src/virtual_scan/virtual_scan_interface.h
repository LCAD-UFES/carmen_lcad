
#ifndef _CARMEN_VIRTUAL_SCAN_INTERFACE_H_
#define _CARMEN_VIRTUAL_SCAN_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <carmen/virtual_scan_messages.h>

carmen_virtual_scan_message *carmen_virtual_scan_new_message(void);

void carmen_virtual_scan_init_message(carmen_virtual_scan_message *message);

void carmen_virtual_scan_clear_message(carmen_virtual_scan_message *message);

#define carmen_virtual_scan_delete_message(MESSAGE) if (MESSAGE != NULL) {carmen_virtual_scan_clear_message(MESSAGE); MESSAGE = NULL;}

void carmen_virtual_scan_set_message(carmen_virtual_scan_message *message, carmen_velodyne_partial_scan_message *velodyne_message);

void carmen_virtual_scan_subscribe_message(carmen_virtual_scan_message *message,
										   carmen_handler_t handler,
										   carmen_subscribe_t subscribe_how);

void carmen_virtual_scan_publish_message(carmen_virtual_scan_message *message);

void carmen_virtual_scan_unsubscribe_message(carmen_handler_t handler);

void carmen_virtual_scan_define_messages(void);

void carmen_virtual_scan_install_params(int argc, char *argv[]);

void carmen_virtual_scan_subscribe_messages(void);

#ifdef __cplusplus
}
#endif

#endif

// @}

/*
 * extra_keys_interface.h
 *
 *  Created on: 3 de ago. de 2022
 *      Author: lume
 */

#ifndef EXTRA_KEYS_INTERFACE_H_
#define EXTRA_KEYS_INTERFACE_H_

#include <carmen/extra_keys_messages.h>

#ifdef __cplusplus
extern "C"
{
#endif

// SUBSCRIBES
void carmen_extra_keys_subscribe(carmen_extra_keys_message_t *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

// UNSUBSCRIBES
void carmen_extra_keys_unsubscribe(carmen_handler_t handler);

void carmen_extra_keys_define_messages();

// PUBLISHES
IPC_RETURN_TYPE carmen_extra_keys_publish_message(carmen_extra_keys_message_t *message);

#ifdef __cplusplus
}
#endif

#endif /* EXTRA_KEYS_INTERFACE_H_ */

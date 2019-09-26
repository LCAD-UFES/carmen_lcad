#ifndef JOYSTICK_INTERFACE_H_
#define JOYSTICK_INTERFACE_H_

#include "joystick_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_joystick_subscribe_status_message(carmen_joystick_status_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_joystick_unsubscribe_status_message(carmen_handler_t handler);

void
carmen_joystick_define_messages();

#ifdef __cplusplus
}
#endif


#endif /* JOYSTICK_INTERFACE_H_ */

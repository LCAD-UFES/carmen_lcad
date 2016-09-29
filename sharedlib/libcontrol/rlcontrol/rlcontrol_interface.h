 
#ifndef CARMEN_RL_CONTROL_INTERFACE_H
#define CARMEN_RL_CONTROL_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/rlcontrol_messages.h>

void
carmen_rl_control_subscribe_message(carmen_rl_control_message *message,
					carmen_handler_t handler,
					carmen_subscribe_t subscribe_how);

void
carmen_rl_control_unsubscribe_message(carmen_handler_t handler);

#ifdef __cplusplus
}
#endif

#endif


#ifndef CARMEN_PI_NIT_INTERFACE_H
#define CARMEN_PI_NIT_INTERFACE_H

#include <carmen/carmen.h>
#include <carmen/pi_nit_messages.h>

#ifdef __cplusplus
extern "C"
{
#endif

IPC_RETURN_TYPE
pi_nit_define_status_message(void);

void
pi_nit_subscribe_status_message(carmen_pi_nit_status_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
pi_nit_unsubscribe_status_message(carmen_handler_t handler);

IPC_RETURN_TYPE
pi_nit_publish_status_message(carmen_pi_nit_status_message *message);

#ifdef __cplusplus
}
#endif

#endif

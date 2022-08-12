#ifndef CARMEN_AUDIT_INTERFACE_H_
#define CARMEN_AUDIT_INTERFACE_H_


#include <carmen/carmen.h>
//#include <carmen/audit_messages.h>
#include "audit_messages.h"

#ifdef __cplusplus
extern "C"
{
#endif

void
carmen_audit_subscribe_operation_message(carmen_audit_operation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_audit_unsubscribe_operation_message(carmen_handler_t handler);

void
carmen_audit_define_messages();

void
carmen_audit_publish_operation_message(carmen_audit_operation_message *audit_operation_message);

void
carmen_audit_publish_status_message(carmen_audit_status_message *audit_status_message);

void
carmen_audit_subscribe_status_message(carmen_audit_status_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_audit_unsubscribe_status_message(carmen_handler_t handler);

#ifdef __cplusplus
}
#endif

#endif

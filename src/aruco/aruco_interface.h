#ifndef ARUCO_INTERFACE_H
#define ARUCO_INTERFACE_H

#include <carmen/carmen.h>
#include "aruco_messages.h"

#ifdef __cplusplus
extern "C"
{
#endif


IPC_RETURN_TYPE 
aruco_define_message(int detector_id);

void
aruco_subscribe_message(int detector_id, carmen_aruco_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

IPC_RETURN_TYPE
aruco_publish_message(int detector_id, const carmen_aruco_message *message);

void
aruco_unsubscribe_message(int detector_id, carmen_handler_t handler);


#ifdef __cplusplus
}
#endif

#endif
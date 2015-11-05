 
#ifndef CARMEN_XSENSODOMETER_INTERFACE_H
#define CARMEN_XSENSODOMETER_INTERFACE_H

#include "xsensOdometer_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_xsensOdometer_subscribe_xsensOdometer_message(	carmen_xsens_odometry_message *xsens_odometry,
					     		carmen_handler_t handler,
					     		carmen_subscribe_t subscribe_how);

void
carmen_xsensOdometer_unsubscribe_xsensOdometer_message(carmen_handler_t handler);


#ifdef __cplusplus
}
#endif

#endif

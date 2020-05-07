#ifndef CAMERA_DRIVERS_INTERFACE_H
#define CAMERA_DRIVERS_INTERFACE_H

#include <carmen/carmen.h>
#include "camera_drivers_messages.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum {
	char_data,
	unsigned_char_data,
	signed_char_data,
	int_data,
	unsigned_int_data,
	short_data,
	unsigned_short_data,
	long_data,
	unsigned_long_data,
	float_data,
	double_data,
	long_double_data,
} camera_image_data_types;


IPC_RETURN_TYPE 
camera_drivers_define_message(int camera_id);

void
camera_drivers_subscribe_message(int camera_id, camera_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

IPC_RETURN_TYPE
camera_drivers_publish_message(int camera_id, const camera_message *message);

void
camera_drivers_unsubscribe_message(int camera_id, carmen_handler_t handler);


#ifdef __cplusplus
}
#endif

#endif
#ifndef CARMEN_PROB_INTERFACE_H
#define CARMEN_PROB_INTERFACE_H

#include <carmen/carmen.h>
#include <carmen/global.h>
#include "prob_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

	void
	carmen_prob_initialize_gaussian_position(carmen_point_t mean, carmen_point_t std);

	void carmen_prob_initialize_uniform_position(void);

	void
	carmen_prob_subscribe_initialize_message(carmen_prob_initialize_message *message,
			carmen_handler_t handler,
			carmen_subscribe_t subscribe_how);

	void
	carmen_prob_unsubscribe_initialize_message(carmen_handler_t handler);

#ifdef __cplusplus
}
#endif

#endif

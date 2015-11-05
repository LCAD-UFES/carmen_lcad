#ifndef SALIENCY_SEARCH_INTERFACE_H_
#define SALIENCY_SEARCH_INTERFACE_H_

#include <carmen/carmen.h>
#include <carmen/fused_odometry_interface.h>
#include "saliency_search_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_saliency_search_subscribe_saliency_points(carmen_saliency_search_saliency_points_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how);

void
carmen_saliency_search_unsubscribe_saliency_points(carmen_handler_t handler);

void
carmen_saliency_define_saliency_points_message();

#ifdef __cplusplus
}
#endif

#endif /* SALIENCY_SEARCH_INTERFACE_H_ */

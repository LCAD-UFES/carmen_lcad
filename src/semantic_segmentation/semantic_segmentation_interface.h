#include <carmen/semantic_segmentation_messages.h>

#ifndef SEMANTIC_SEGMENTIATION_INTERFACE_H
#define SEMANTIC_SEGMENTIATION_INTERFACE_H

void
carmen_semantic_segmentation_subscribe_image_message(carmen_semantic_segmentation_image_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_semantic_segmentation_unsubscribe_image_message(carmen_handler_t handler);

void
carmen_semantic_segmentation_define_messages();

#endif

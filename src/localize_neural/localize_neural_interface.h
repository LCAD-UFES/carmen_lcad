#ifndef CARMEN_LOCALIZE_NEURAL_INTERFACE_H
#define CARMEN_LOCALIZE_NEURAL_INTERFACE_H

#include "localize_neural_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void 
carmen_localize_neural_subscribe_imagepos_keyframe_message(carmen_localize_neural_imagepos_message
					   *imagepos, carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how);

void
carmen_localize_neural_unsubscribe_imagepos_keyframe_message(carmen_handler_t handler);

void
carmen_localize_neural_publish_imagepos_keyframe_message(carmen_localize_neural_imagepos_message *message);

void
carmen_localize_neural_subscribe_imagepos_curframe_message(carmen_localize_neural_imagepos_message
					   *imagepos, carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how);

void
carmen_localize_neural_unsubscribe_imagepos_curframe_message(carmen_handler_t handler);

void
carmen_localize_neural_publish_imagepos_curframe_message(carmen_localize_neural_imagepos_message *message);

#ifdef __cplusplus
}
#endif

#endif
// @}

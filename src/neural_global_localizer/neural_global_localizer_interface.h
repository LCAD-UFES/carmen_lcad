#include <carmen/neural_global_localizer_messages.h>

#ifndef CARMEN_NEURAL_GLOBAL_LOCALIZER_INTERFACE_H
#define CARMEN_NEURAL_GLOBAL_LOCALIZER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

void carmen_neural_global_localizer_subscribe_globalpos_message(carmen_neural_global_localizer_globalpos_message *globalpos,
																carmen_handler_t handler,
																carmen_subscribe_t subscribe_how);

void carmen_neural_global_localizer_unsubscribe_globalpos_message(carmen_handler_t handler);

void carmen_neural_global_localizer_define_messages();

#ifdef __cplusplus
}
#endif

#endif

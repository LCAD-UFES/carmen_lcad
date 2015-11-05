#include <carmen/neural_localizer_messages.h>

#ifdef __cplusplus
extern "C" {
#endif


void
carmen_neural_localizer_subscribe_place_command_message(carmen_neural_localizer_place_command_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how);

void
carmen_neural_localizer_unsubscribe_place_command_message(carmen_handler_t handler);

void
carmen_neural_localizer_define_messages();

#ifdef __cplusplus
}
#endif


// @}


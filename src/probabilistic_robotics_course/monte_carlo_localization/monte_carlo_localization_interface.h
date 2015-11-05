#include "monte_carlo_localization_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_monte_carlo_localization_subscribe_message(carmen_monte_carlo_localization_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_monte_carlo_localization_unsubscribe_message(carmen_handler_t handler);

void
carmen_monte_carlo_localization_define_messages();

#ifdef __cplusplus
}
#endif


// @}


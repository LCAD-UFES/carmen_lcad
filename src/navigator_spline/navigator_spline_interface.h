#include "navigator_spline_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_navigator_spline_subscribe_path_message(carmen_navigator_spline_path_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_navigator_spline_unsubscribe_path_message(carmen_handler_t handler);

#ifdef __cplusplus
}
#endif


// @}


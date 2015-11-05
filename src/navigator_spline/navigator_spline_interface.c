#include <carmen/carmen.h>
#include "navigator_spline_messages.h"

void
carmen_navigator_spline_subscribe_path_message(carmen_navigator_spline_path_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_NAVIGATOR_SPLINE_PATH_NAME, 
                           CARMEN_NAVIGATOR_SPLINE_PATH_FMT,
                           message, sizeof(carmen_navigator_spline_path_message), 
			   handler, subscribe_how);
}


void
carmen_navigator_spline_unsubscribe_path_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_NAVIGATOR_SPLINE_PATH_NAME, handler);
}

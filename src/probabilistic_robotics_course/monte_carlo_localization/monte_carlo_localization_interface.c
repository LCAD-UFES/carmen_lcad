#include <carmen/carmen.h>
#include "monte_carlo_localization_interface.h"

void
carmen_monte_carlo_localization_subscribe_message(carmen_monte_carlo_localization_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_MONTE_CARLO_LOCALIZATION_NAME,
		  CARMEN_MONTE_CARLO_LOCALIZATION_MESSAGE_FMT,
                           message, sizeof(carmen_monte_carlo_localization_message),
			   handler, subscribe_how);
}


void
carmen_monte_carlo_localization_unsubscribe_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_MONTE_CARLO_LOCALIZATION_NAME, handler);
}


void
carmen_monte_carlo_localization_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_MONTE_CARLO_LOCALIZATION_NAME, IPC_VARIABLE_LENGTH,
		  CARMEN_MONTE_CARLO_LOCALIZATION_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_MONTE_CARLO_LOCALIZATION_NAME);
}


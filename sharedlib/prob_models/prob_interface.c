#include "prob_interface.h"
#include "prob_messages.h"

void
carmen_prob_initialize_gaussian_position(carmen_point_t mean, carmen_point_t std)
{
  static carmen_prob_initialize_message init;
  static int first = 1;
  IPC_RETURN_TYPE err;

  if(first)
  {
	  err = IPC_defineMsg(CARMEN_PROB_INITIALIZE_NAME,
			  IPC_VARIABLE_LENGTH,
			  CARMEN_PROB_INITIALIZE_FMT);
	  carmen_test_ipc_exit(err, "Could not define message",
			  CARMEN_PROB_INITIALIZE_NAME);

	  first = 0;
  }
  init.timestamp = carmen_get_time();
  init.host = carmen_get_host();

  init.distribution_type = CARMEN_PROB_INITIALIZE_GAUSSIAN;
  init.distribution_modes = 1;
  init.mean = &mean;
  init.std = &std;
  err = IPC_publishData(CARMEN_PROB_INITIALIZE_NAME, &init);
  carmen_test_ipc(err, "Could not publish", CARMEN_PROB_INITIALIZE_NAME);
}

void carmen_prob_initialize_uniform_position(void)
{
  static carmen_prob_initialize_message init;
  static int first = 1;
  IPC_RETURN_TYPE err;

  if(first)
  {
	  err = IPC_defineMsg(CARMEN_PROB_INITIALIZE_NAME,
			  IPC_VARIABLE_LENGTH,
			  CARMEN_PROB_INITIALIZE_FMT);
	  carmen_test_ipc_exit(err, "Could not define message",
			  CARMEN_PROB_INITIALIZE_NAME);

	  first = 0;
  }
  init.timestamp = carmen_get_time();
  init.host = carmen_get_host();

  init.distribution_type = CARMEN_PROB_INITIALIZE_UNIFORM;
  init.distribution_modes = 0;
  init.mean = NULL;
  init.std = NULL;
  err = IPC_publishData(CARMEN_PROB_INITIALIZE_NAME, &init);
  carmen_test_ipc(err, "Could not publish", CARMEN_PROB_INITIALIZE_NAME);
}

void
carmen_prob_subscribe_initialize_message(carmen_prob_initialize_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(
			CARMEN_PROB_INITIALIZE_NAME,
			CARMEN_PROB_INITIALIZE_FMT,
			message, sizeof(carmen_prob_initialize_message), handler, subscribe_how);
}

void
carmen_prob_unsubscribe_initialize_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_PROB_INITIALIZE_NAME, handler);
}


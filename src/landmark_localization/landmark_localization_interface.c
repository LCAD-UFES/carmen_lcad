#include <carmen/carmen.h>
#include <carmen/landmark_localization_messages.h>

void
carmen_landmark_localization_subscribe_state_message(carmen_landmark_localization_state_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_LANDMARK_LOCALIZATION_STATE_MESSAGE_NAME,
		  	  	  	  	   CARMEN_LANDMARK_LOCALIZATION_STATE_MESSAGE_FMT,
                           message, sizeof(carmen_landmark_localization_state_message),
                           handler, subscribe_how);
}


void
carmen_landmark_localization_unsubscribe_state_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_LANDMARK_LOCALIZATION_STATE_MESSAGE_NAME, handler);
}


void
carmen_landmark_localization_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_LANDMARK_LOCALIZATION_STATE_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
		  CARMEN_LANDMARK_LOCALIZATION_STATE_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LANDMARK_LOCALIZATION_STATE_MESSAGE_NAME);
}


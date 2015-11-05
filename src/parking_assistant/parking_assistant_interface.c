#include <carmen/carmen.h>
#include <carmen/parking_assistant_messages.h>

void
carmen_parking_assistant_subscribe_goal(carmen_parking_assistant_goal_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_PARKING_ASSISTANT_GOAL_NAME,
                           CARMEN_PARKING_ASSISTANT_GOAL_FMT,
                           message, sizeof(carmen_parking_assistant_goal_message),
			   handler, subscribe_how);
}


void
carmen_parking_assistant_unsubscribe_goal(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_PARKING_ASSISTANT_GOAL_NAME, handler);
}

void
carmen_parking_assistant_subscribe_parking_space(carmen_parking_assistant_parking_space_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_PARKING_ASSISTANT_PARKING_SPACE_NAME,
                           CARMEN_PARKING_ASSISTANT_PARKING_SPACE_FMT,
                           message, sizeof(carmen_parking_assistant_parking_space_message),
			   handler, subscribe_how);
}


void
carmen_parking_assistant_unsubscribe_parking_space(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_PARKING_ASSISTANT_PARKING_SPACE_NAME, handler);
}


void
carmen_parking_assistant_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_PARKING_ASSISTANT_GOAL_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARKING_ASSISTANT_GOAL_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_PARKING_ASSISTANT_GOAL_NAME);

  err = IPC_defineMsg(CARMEN_PARKING_ASSISTANT_PARKING_SPACE_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARKING_ASSISTANT_PARKING_SPACE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_PARKING_ASSISTANT_PARKING_SPACE_NAME);
}


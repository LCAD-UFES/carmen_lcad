#include <carmen/carmen.h>
#include <carmen/tracker_messages.h>
#include <carmen/tracker_interface.h>

//subscribes to the trainning message
void
carmen_tracker_subscribe_train(
		carmen_tracker_train_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_TRACKER_TRAINING_MESSAGE_NAME,
                           CARMEN_TRACKER_TRAINING_MESSAGE_FMT,
                           message, sizeof(carmen_tracker_train_message),
			   handler, subscribe_how);
  printf("\nSubscribe to Visual Search Thin ! - Training Mode\n");
}

//unsubscribe to the trainning message
void
carmen_tracker_unsubscribe_train(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_TRACKER_TRAINING_MESSAGE_NAME, handler);
}

//subscribes to the test message
void
carmen_tracker_subscribe_test(
		carmen_tracker_test_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_TRACKER_TEST_MESSAGE_NAME,
                           CARMEN_TRACKER_TEST_MESSAGE_FMT,
                           message, sizeof(carmen_tracker_test_message),
			   handler, subscribe_how);
  printf("\nSubscribe to Visual Search Thin ! - Testing Mode\n");
}

//unsubscribe to the test message
void
carmen_tracker_unsubscribe_test(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_TRACKER_TEST_MESSAGE_NAME, handler);
}

//subscribes to the output message
void
carmen_tracker_subscribe_output(
		carmen_tracker_output_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_TRACKER_OUTPUT_MESSAGE_NAME,
                           CARMEN_TRACKER_OUTPUT_MESSAGE_FMT,
                           message, sizeof(carmen_tracker_output_message),
			   handler, subscribe_how);
  printf("\nSubscribe to Visual Search Thin ! - Output Mode\n");
}

//unsubscribe to the output message
void
carmen_tracker_unsubscribe_output(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_TRACKER_OUTPUT_MESSAGE_NAME, handler);
}

IPC_RETURN_TYPE
carmen_tracker_define_message_train(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_TRACKER_TRAINING_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_TRACKER_TRAINING_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_TRACKER_TRAINING_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_tracker_define_message_test(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_TRACKER_TEST_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_TRACKER_TEST_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_TRACKER_TEST_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_tracker_define_message_output(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_TRACKER_OUTPUT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_TRACKER_OUTPUT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_TRACKER_OUTPUT_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_tracker_define_message_query_test(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_TRACKER_QUERY_TEST_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_TRACKER_QUERY_TEST_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_TRACKER_QUERY_TEST_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_tracker_define_message_position()
{
  IPC_RETURN_TYPE err;
  err = IPC_defineMsg(CARMEN_TRACKER_POSITION_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_TRACKER_POSITION_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_TRACKER_POSITION_MESSAGE_NAME);
  return(err);
}

void
carmen_tracker_subscribe_query_test(HANDLER_TYPE handler)
{
	IPC_RETURN_TYPE err;
	err = IPC_subscribe(CARMEN_TRACKER_QUERY_TEST_MESSAGE_NAME, handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_TRACKER_QUERY_TEST_MESSAGE_NAME);
	IPC_setMsgQueueLength(CARMEN_TRACKER_QUERY_TEST_MESSAGE_NAME, 1);
}

carmen_tracker_output_message *
carmen_tracker_query_output_message(carmen_tracker_test_message *query, double timeout)
{
	IPC_RETURN_TYPE err;
	carmen_tracker_output_message *response = 0;
	unsigned int timeoutMsecs = (unsigned int) timeout * 1000.0;

	err = IPC_queryResponseData(CARMEN_TRACKER_QUERY_TEST_MESSAGE_NAME, query, (void **)&response, timeoutMsecs);

	if (err != IPC_OK)
	{
		carmen_test_ipc(err, "Could not get visual search output", CARMEN_TRACKER_QUERY_TEST_MESSAGE_NAME);
		return NULL;
	}

	return(response);

}

IPC_RETURN_TYPE
carmen_tracker_define_message_query_train(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_TRACKER_QUERY_TRAINING_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_TRACKER_QUERY_TRAINING_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_TRACKER_QUERY_TRAINING_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_tracker_define_message_output_train(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_TRACKER_OUTPUT_TRAINING_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_TRACKER_OUTPUT_TRAINING_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_TRACKER_OUTPUT_TRAINING_MESSAGE_NAME);
	return(err);
}

void
carmen_tracker_subscribe_query_train(HANDLER_TYPE handler)
{
	IPC_RETURN_TYPE err;
	err = IPC_subscribe(CARMEN_TRACKER_QUERY_TRAINING_MESSAGE_NAME, handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_TRACKER_QUERY_TRAINING_MESSAGE_NAME);
	IPC_setMsgQueueLength(CARMEN_TRACKER_QUERY_TRAINING_MESSAGE_NAME, 1);
}

void
carmen_tracker_subscribe_position_message(carmen_tracker_position_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_TRACKER_POSITION_MESSAGE_NAME, CARMEN_TRACKER_POSITION_MESSAGE_FMT, message, sizeof(carmen_tracker_position_message), handler, subscribe_how);
}


carmen_tracker_output_training_message *
carmen_tracker_query_training_message(carmen_tracker_train_message *query, double timeout)
{
	IPC_RETURN_TYPE err;
	carmen_tracker_output_training_message *response = 0;
	unsigned int timeoutMsecs = (unsigned int) timeout * 1000.0;

	err = IPC_queryResponseData(CARMEN_TRACKER_QUERY_TRAINING_MESSAGE_NAME, query, (void **)&response, timeoutMsecs);

	if (err != IPC_OK)
	{
		carmen_test_ipc(err, "Could not get visual search training output", CARMEN_TRACKER_QUERY_TRAINING_MESSAGE_NAME);
		return NULL;
	}

	return(response);

}

void
position_message_handler(double x, double y, double timestamp)
{
	carmen_tracker_position_message position_message;
	position_message.object_position.x = x;
	position_message.object_position.y = y;
	position_message.host = carmen_get_host();
	position_message.timestamp = timestamp;

	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_TRACKER_POSITION_MESSAGE_NAME, &position_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_TRACKER_POSITION_MESSAGE_NAME);
}

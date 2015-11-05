#include <carmen/carmen.h>
#include <carmen/visual_search_thin_messages.h>
#include <carmen/visual_search_thin_interface.h>

//subscribes to the trainning message
void
carmen_visual_search_thin_subscribe_train(
		carmen_visual_search_thin_train_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_VISUAL_SEARCH_THIN_TRAINING_MESSAGE_NAME,
                           CARMEN_VISUAL_SEARCH_THIN_TRAINING_MESSAGE_FMT,
                           message, sizeof(carmen_visual_search_thin_train_message),
			   handler, subscribe_how);
  printf("\nSubscribe to Visual Search Thin ! - Training Mode\n");
}

//unsubscribe to the trainning message
void
carmen_visual_search_thin_unsubscribe_train(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_VISUAL_SEARCH_THIN_TRAINING_MESSAGE_NAME, handler);
}

//subscribes to the test message
void
carmen_visual_search_thin_subscribe_test(
		carmen_visual_search_thin_test_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_VISUAL_SEARCH_THIN_TEST_MESSAGE_NAME,
                           CARMEN_VISUAL_SEARCH_THIN_TEST_MESSAGE_FMT,
                           message, sizeof(carmen_visual_search_thin_test_message),
			   handler, subscribe_how);
  printf("\nSubscribe to Visual Search Thin ! - Testing Mode\n");
}

//unsubscribe to the test message
void
carmen_visual_search_thin_unsubscribe_test(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_VISUAL_SEARCH_THIN_TEST_MESSAGE_NAME, handler);
}

//subscribes to the output message
void
carmen_visual_search_thin_subscribe_output(
		carmen_visual_search_thin_output_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_VISUAL_SEARCH_THIN_OUTPUT_MESSAGE_NAME,
                           CARMEN_VISUAL_SEARCH_THIN_OUTPUT_MESSAGE_FMT,
                           message, sizeof(carmen_visual_search_thin_output_message),
			   handler, subscribe_how);
  printf("\nSubscribe to Visual Search Thin ! - Output Mode\n");
}

//unsubscribe to the output message
void
carmen_visual_search_thin_unsubscribe_output(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_VISUAL_SEARCH_THIN_OUTPUT_MESSAGE_NAME, handler);
}

IPC_RETURN_TYPE
carmen_visual_search_thin_define_message_train(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_SEARCH_THIN_TRAINING_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VISUAL_SEARCH_THIN_TRAINING_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_SEARCH_THIN_TRAINING_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_visual_search_thin_define_message_test(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_SEARCH_THIN_TEST_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VISUAL_SEARCH_THIN_TEST_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_SEARCH_THIN_TEST_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_visual_search_thin_define_message_output(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_SEARCH_THIN_OUTPUT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VISUAL_SEARCH_THIN_OUTPUT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_SEARCH_THIN_OUTPUT_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_visual_search_thin_define_message_query_test(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_SEARCH_THIN_QUERY_TEST_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VISUAL_SEARCH_THIN_QUERY_TEST_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_SEARCH_THIN_QUERY_TEST_MESSAGE_NAME);
	return(err);
}

void
carmen_visual_search_thin_subscribe_query_test(HANDLER_TYPE handler)
{
	IPC_RETURN_TYPE err;
	err = IPC_subscribe(CARMEN_VISUAL_SEARCH_THIN_QUERY_TEST_MESSAGE_NAME, handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_VISUAL_SEARCH_THIN_QUERY_TEST_MESSAGE_NAME);
	IPC_setMsgQueueLength(CARMEN_VISUAL_SEARCH_THIN_QUERY_TEST_MESSAGE_NAME, 1);
}

carmen_visual_search_thin_output_message *
carmen_visual_search_thin_query_output_message(carmen_visual_search_thin_test_message *query, double timeout)
{
	IPC_RETURN_TYPE err;
	carmen_visual_search_thin_output_message *response = 0;
	unsigned int timeoutMsecs = (unsigned int) timeout * 1000.0;

	err = IPC_queryResponseData(CARMEN_VISUAL_SEARCH_THIN_QUERY_TEST_MESSAGE_NAME, query, (void **)&response, timeoutMsecs);

	if (err != IPC_OK)
	{
		carmen_test_ipc(err, "Could not get visual search output", CARMEN_VISUAL_SEARCH_THIN_QUERY_TEST_MESSAGE_NAME);
		return NULL;
	}

	return(response);

}

IPC_RETURN_TYPE
carmen_visual_search_thin_define_message_query_train(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_SEARCH_THIN_QUERY_TRAINING_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VISUAL_SEARCH_THIN_QUERY_TRAINING_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_SEARCH_THIN_QUERY_TRAINING_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_visual_search_thin_define_message_output_train(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_SEARCH_THIN_OUTPUT_TRAINING_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VISUAL_SEARCH_THIN_OUTPUT_TRAINING_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_SEARCH_THIN_OUTPUT_TRAINING_MESSAGE_NAME);
	return(err);
}

void
carmen_visual_search_thin_subscribe_query_train(HANDLER_TYPE handler)
{
	IPC_RETURN_TYPE err;
	err = IPC_subscribe(CARMEN_VISUAL_SEARCH_THIN_QUERY_TRAINING_MESSAGE_NAME, handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_VISUAL_SEARCH_THIN_QUERY_TRAINING_MESSAGE_NAME);
	IPC_setMsgQueueLength(CARMEN_VISUAL_SEARCH_THIN_QUERY_TRAINING_MESSAGE_NAME, 1);
}


carmen_visual_search_thin_output_training_message *
carmen_visual_search_thin_query_training_message(carmen_visual_search_thin_train_message *query, double timeout)
{
	IPC_RETURN_TYPE err;
	carmen_visual_search_thin_output_training_message *response = 0;
	unsigned int timeoutMsecs = (unsigned int) timeout * 1000.0;

	err = IPC_queryResponseData(CARMEN_VISUAL_SEARCH_THIN_QUERY_TRAINING_MESSAGE_NAME, query, (void **)&response, timeoutMsecs);

	if (err != IPC_OK)
	{
		carmen_test_ipc(err, "Could not get visual search training output", CARMEN_VISUAL_SEARCH_THIN_QUERY_TRAINING_MESSAGE_NAME);
		return NULL;
	}

	return(response);

}

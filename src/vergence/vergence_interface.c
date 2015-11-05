#include <carmen/carmen.h>
#include <carmen/vergence_messages.h>
#include <carmen/vergence_interface.h>

IPC_RETURN_TYPE
carmen_vergence_define_message_train(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VERGENCE_TRAIN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VERGENCE_TRAIN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VERGENCE_TRAIN_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_vergence_define_message_test(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VERGENCE_TEST_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VERGENCE_TEST_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VERGENCE_TEST_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_vergence_define_message_output_test(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VERGENCE_TEST_OUTPUT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VERGENCE_TEST_OUTPUT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VERGENCE_TEST_OUTPUT_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_vergence_define_message_output_train(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VERGENCE_TRAIN_OUTPUT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VERGENCE_TRAIN_OUTPUT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VERGENCE_TRAIN_OUTPUT_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_vergence_define_message_query_train(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VERGENCE_QUERY_TRAIN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VERGENCE_QUERY_TRAIN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VERGENCE_QUERY_TRAIN_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_vergence_define_message_query_test(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VERGENCE_QUERY_TEST_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VERGENCE_QUERY_TEST_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VERGENCE_QUERY_TEST_MESSAGE_NAME);
	return(err);
}

void
carmen_vergence_subscribe_query_train(HANDLER_TYPE handler)
{
	IPC_RETURN_TYPE err;
	err = IPC_subscribe(CARMEN_VERGENCE_QUERY_TRAIN_MESSAGE_NAME, handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_VERGENCE_QUERY_TRAIN_MESSAGE_NAME);
	IPC_setMsgQueueLength(CARMEN_VERGENCE_QUERY_TRAIN_MESSAGE_NAME, 1);
}

void
carmen_vergence_subscribe_query_test(HANDLER_TYPE handler)
{
	IPC_RETURN_TYPE err;
	err = IPC_subscribe(CARMEN_VERGENCE_QUERY_TEST_MESSAGE_NAME, handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_VERGENCE_QUERY_TEST_MESSAGE_NAME);
	IPC_setMsgQueueLength(CARMEN_VERGENCE_QUERY_TEST_MESSAGE_NAME, 1);
}

carmen_vergence_train_output_message *
carmen_vergence_query_train_message(carmen_vergence_train_message *query, double timeout)
{
	IPC_RETURN_TYPE err;
	carmen_vergence_train_output_message *response = 0;
	unsigned int timeoutMsecs = (unsigned int) timeout * 1000.0;

	err = IPC_queryResponseData(CARMEN_VERGENCE_QUERY_TRAIN_MESSAGE_NAME, query, (void **)&response, timeoutMsecs);

	if (err != IPC_OK)
	{
		carmen_test_ipc(err, "Could not get vergence train output", CARMEN_VERGENCE_QUERY_TRAIN_MESSAGE_NAME);
		return NULL;
	}

	return(response);

}

carmen_vergence_test_output_message *
carmen_vergence_query_test_message(carmen_vergence_test_message *query, double timeout)
{
	IPC_RETURN_TYPE err;
	carmen_vergence_test_output_message *response = 0;
	unsigned int timeoutMsecs = (unsigned int) timeout * 1000.0;

	err = IPC_queryResponseData(CARMEN_VERGENCE_QUERY_TEST_MESSAGE_NAME, query, (void **)&response, timeoutMsecs);

	if (err != IPC_OK)
	{
		carmen_test_ipc(err, "Could not get vergence test output", CARMEN_VERGENCE_QUERY_TEST_MESSAGE_NAME);
		return NULL;
	}

	return(response);

}

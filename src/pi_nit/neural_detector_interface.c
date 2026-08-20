#include <carmen/carmen.h>
#include <carmen/neural_detector_interface.h>


char*
set_message(int message_id)
{
	char *message_name = (char*) malloc (64 * sizeof(char));
	sprintf(message_name, "neural_detector_message_%d_name", message_id);
	return message_name;
}


IPC_RETURN_TYPE
neural_detector_define_message(int message_id)
{
    IPC_RETURN_TYPE err;
	char *message_name = set_message(message_id);
	
	err = IPC_defineMsg(message_name, IPC_VARIABLE_LENGTH, NEURAL_DETECTOR_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", message_name);
	free(message_name);
	return (err);
}


void
neural_detector_subscribe_message(int message_id, neural_detector_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    char *message_name = set_message(message_id);

	carmen_subscribe_message(message_name, NEURAL_DETECTOR_MESSAGE_FMT, message, sizeof(neural_detector_message),
		                    handler, subscribe_how);
	free(message_name);
}


IPC_RETURN_TYPE
neural_detector_publish_message(int message_id, neural_detector_message *neural_object_tracker_message)
{
    IPC_RETURN_TYPE err;
	char *message_name = set_message(message_id);

	err = IPC_publishData(message_name, neural_object_tracker_message);
	carmen_test_ipc_exit(err, "Could not publish", message_name);
	free(message_name);
	return (err);
}


void
neural_detector_unsubscribe_message(int message_id, carmen_handler_t handler)
{
    char *message_name = set_message(message_id);
	
	carmen_unsubscribe_message(message_name, handler);
	free(message_name);
}

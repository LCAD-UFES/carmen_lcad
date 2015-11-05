#include <carmen/carmen.h>
#include <carmen/visual_memory_messages.h>
#include <carmen/visual_memory_interface.h>



//subscribes to the trainning message

void
carmen_visual_memory_subscribe_train(carmen_visual_memory_message *visual_memory_message,
                                     carmen_handler_t handler,
                                     carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_VISUAL_MEMORY_TRAINING_MESSAGE_NAME,
                             CARMEN_VISUAL_MEMORY_TRAINING_MESSAGE_FMT,
                             visual_memory_message, sizeof (carmen_visual_memory_message),
                             handler, subscribe_how);
    printf("\nSubscribe to Visual Memory ! - Trainning Mode\n");
}

//unsubscribe to the trainning message

void
carmen_visual_memory_unsubscribe_train(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_VISUAL_MEMORY_TRAINING_MESSAGE_NAME, handler);
}

IPC_RETURN_TYPE
carmen_visual_memory_define_message_train(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_MEMORY_TRAINING_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VISUAL_MEMORY_TRAINING_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_MEMORY_TRAINING_MESSAGE_NAME);
	return(err);
}

IPC_RETURN_TYPE
carmen_visual_memory_define_message_output(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_VISUAL_MEMORY_OUTPUT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VISUAL_MEMORY_OUTPUT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VISUAL_MEMORY_OUTPUT_MESSAGE_NAME);
	return(err);
}
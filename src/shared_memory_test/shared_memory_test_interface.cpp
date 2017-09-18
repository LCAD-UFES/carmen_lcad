#include <carmen/carmen.h>
#include "shared_memory_test_messages.h"

void
carmen_shared_memory_test_subscribe_map_message(carmen_shared_memory_test_map_message
		*msg, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char *)CARMEN_SHARED_MEMORY_TEST_MAP_MESSAGE_NAME,
			(char *)CARMEN_SHARED_MEMORY_TEST_MAP_MESSAGE_FMT,
			msg, sizeof(carmen_shared_memory_test_map_message),
			handler, subscribe_how);
}

void
carmen_shared_memory_test_unsubscribe_map_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char *)CARMEN_SHARED_MEMORY_TEST_MAP_MESSAGE_NAME, handler);
}


void
carmen_shared_memory_test_publish_map_message(carmen_shared_memory_test_map_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_SHARED_MEMORY_TEST_MAP_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_SHARED_MEMORY_TEST_MAP_MESSAGE_NAME);
}



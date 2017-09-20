#ifndef CARMEN_SHARED_MEMORY_TEST_INTERFACE_H
#define CARMEN_SHARED_MEMORY_TEST_INTERFACE_H

#include "shared_memory_test_messages.h"



void
carmen_shared_memory_test_subscribe_map_message(carmen_shared_memory_test_map_message
		*msg, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void
carmen_shared_memory_test_unsubscribe_map_message(carmen_handler_t handler);


void
carmen_shared_memory_test_publish_map_message(carmen_shared_memory_test_map_message *message);



#endif
// @}

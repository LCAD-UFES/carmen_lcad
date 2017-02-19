#include "udatmo_interface.h"

#include "udatmo_memory.h"


carmen_udatmo_moving_obstacles_message *carmen_udatmo_new_moving_obstacles_message(size_t size)
{
	carmen_udatmo_moving_obstacles_message *message = CREATE(carmen_udatmo_moving_obstacles_message);
	carmen_udatmo_init_moving_obstacles_message(message, size);
	return message;
}


void carmen_udatmo_init_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message, size_t size)
{
	// The value of carmen_get_host() is retrieved through getenv(), which may change
	// the returned buffer on later calls. Therefore it's safer to create a copy of
	// the returned string.
	//
	// See: http://www.cplusplus.com/reference/cstdlib/getenv/
	//
	// "The pointer returned [by a getenv() call] points to an internal memory block,
	// whose content or validity may be altered by further calls to getenv
	// (but not by other library functions)."
	static char *host = NULL;
	if (host == NULL)
	{
		char *host_temp = carmen_get_host();
		host = (char*) malloc(strlen(host_temp) * sizeof(char));
		strcpy(host, host_temp);
	}

	message->host = host;
	message->timestamp = 0;
	message->num_obstacles = size;

	int bytes = sizeof(carmen_datmo_moving_obstacle) * size;
	message->obstacles = (carmen_datmo_moving_obstacle*) malloc(bytes);
	memset(message->obstacles, 0, bytes);
}


void carmen_udatmo_clear_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message)
{
	message->host = NULL;
	message->timestamp = 0;
	message->num_obstacles = 0;
	DELETE(message->obstacles);
}


void carmen_udatmo_publish_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message)
{
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_NAME);
}


void carmen_udatmo_subscribe_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message,
													  void (*handler)(carmen_udatmo_moving_obstacles_message*),
													  carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char*) CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_NAME,
							 (char*) CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_FMT,
							 message, sizeof(carmen_udatmo_moving_obstacles_message),
							 (carmen_handler_t) handler,
							 subscribe_how);
}


void carmen_udatmo_unsubscribe_moving_obstacles_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char*) CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_NAME, handler);
}

#include <carmen/carmen.h>
#include <carmen/moving_objects3_messages.h>


void
carmen_subscribe_velodyne_projected_message(carmen_velodyne_projected_on_ground_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_VELODYNE_PROJECTED_MESSAGE_NAME,
			CARMEN_VELODYNE_PROJECTED_MESSAGE_FMT,
			message, sizeof(carmen_velodyne_projected_on_ground_message),
			handler, subscribe_how);
}


void
carmen_publish_velodyne_projected_message(carmen_velodyne_projected_on_ground_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_VELODYNE_PROJECTED_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VELODYNE_PROJECTED_MESSAGE_NAME);
}


void
carmen_unsubscribe_velodyne_projected_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_VELODYNE_PROJECTED_MESSAGE_NAME, handler);
}


void
carmen_subscribe_moving_objects3_particles_message(carmen_moving_objects3_particles_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MOVING_OBJECTS3_PARTICLES_MESSAGE_NAME,
			CARMEN_MOVING_OBJECTS3_PARTICLES_MESSAGE_FMT,
			message, sizeof(carmen_moving_objects3_particles_message),
			handler, subscribe_how);
}


void
carmen_publish_moving_objects3_particles_message(carmen_moving_objects3_particles_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_MOVING_OBJECTS3_PARTICLES_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MOVING_OBJECTS3_PARTICLES_MESSAGE_NAME);
}


void
carmen_unsubscribe_moving_objects3_particles_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_MOVING_OBJECTS3_PARTICLES_MESSAGE_NAME, handler);
}

void
carmen_moving_objects3_define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_VELODYNE_PROJECTED_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_PROJECTED_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_PROJECTED_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_MOVING_OBJECTS3_PARTICLES_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_MOVING_OBJECTS3_PARTICLES_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MOVING_OBJECTS3_PARTICLES_MESSAGE_NAME);
}


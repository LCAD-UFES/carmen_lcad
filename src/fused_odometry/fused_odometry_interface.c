#include <carmen/carmen.h>
#include <carmen/fused_odometry_messages.h>

void
carmen_fused_odometry_subscribe_fused_odometry_message(	carmen_fused_odometry_message *fused_odometry_message,
					carmen_handler_t handler,
					carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_FUSED_ODOMETRY_NAME, 
                           CARMEN_FUSED_ODOMETRY_FMT,
                           fused_odometry_message, sizeof(carmen_fused_odometry_message), 
			   handler, subscribe_how);
}

void
carmen_fused_odometry_unsubscribe_fused_odometry_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_FUSED_ODOMETRY_NAME, handler);
}


void
carmen_fused_odometry_subscribe_fused_odometry_particle_message(	carmen_fused_odometry_particle_message *fused_odometry_particle_message,
									carmen_handler_t handler,
									carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(	CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, 
                           		CARMEN_FUSED_ODOMETRY_PARTICLE_FMT,
                          		fused_odometry_particle_message, sizeof(carmen_fused_odometry_particle_message), 
					handler, subscribe_how);
}

void
carmen_fused_odometry_unsubscribe_fused_odometry_particle_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, handler);
}

void
carmen_fused_odometry_publish_message(carmen_fused_odometry_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_FUSED_ODOMETRY_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_FUSED_ODOMETRY_NAME);
}

void
carmen_fused_odometry_publish_particles(carmen_fused_odometry_particle_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_FUSED_ODOMETRY_PARTICLE_NAME);
}


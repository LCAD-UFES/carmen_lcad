#include <carmen/carmen.h>
#include <carmen/fast_polar_slam_messages.h>


void
carmen_fast_polar_slam_subscribe_best_particle_message(carmen_fast_polar_slam_best_particle_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_FAST_POLAR_SLAM_BEST_PARTICLE_MESSAGE_NAME,
			CARMEN_FAST_POLAR_SLAM_BEST_PARTICLE_MESSAGE_FMT,
			message, sizeof(carmen_fast_polar_slam_best_particle_message),
			handler, subscribe_how);
}


void
carmen_fast_polar_slam_publish_best_particle_message(carmen_fast_polar_slam_best_particle_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_FAST_POLAR_SLAM_BEST_PARTICLE_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_FAST_POLAR_SLAM_BEST_PARTICLE_MESSAGE_NAME);
}


void
carmen_fast_polar_slam_unsubscribe_best_particle_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_FAST_POLAR_SLAM_BEST_PARTICLE_MESSAGE_NAME, handler);
}


void
carmen_fast_polar_slam_subscribe_velodyne_projected_message(carmen_fast_polar_slam_velodyne_projected_on_ground_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_FAST_POLAR_SLAM_VELODYNE_PROJECTED_MESSAGE_NAME,
			CARMEN_FAST_POLAR_SLAM_VELODYNE_PROJECTED_MESSAGE_FMT,
			message, sizeof(carmen_fast_polar_slam_velodyne_projected_on_ground_message),
			handler, subscribe_how);
}

void
carmen_fast_polar_slam_publish_velodyne_projected_message(carmen_fast_polar_slam_velodyne_projected_on_ground_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_FAST_POLAR_SLAM_VELODYNE_PROJECTED_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_FAST_POLAR_SLAM_VELODYNE_PROJECTED_MESSAGE_NAME);
}


void
carmen_fast_polar_slam_unsubscribe_velodyne_projected_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_FAST_POLAR_SLAM_VELODYNE_PROJECTED_MESSAGE_NAME, handler);
}


void
carmen_fast_polar_slam_subscribe_particles_message(carmen_fast_polar_slam_particles_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_FAST_POLAR_SLAM_PARTICLES_MESSAGE_NAME,
			CARMEN_FAST_POLAR_SLAM_PARTICLES_MESSAGE_FMT,
			message, sizeof(carmen_fast_polar_slam_particles_message),
			handler, subscribe_how);
}

void
carmen_fast_polar_slam_publish_particles_message(carmen_fast_polar_slam_particles_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_FAST_POLAR_SLAM_PARTICLES_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_FAST_POLAR_SLAM_PARTICLES_MESSAGE_NAME);
}


void
carmen_fast_polar_slam_unsubscribe_particles_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_FAST_POLAR_SLAM_PARTICLES_MESSAGE_NAME, handler);
}

void
carmen_fast_polar_slam_subscribe_measurement_model_message(carmen_fast_polar_slam_measurement_model_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_FAST_POLAR_SLAM_MEASUREMENT_MODEL_MESSAGE_NAME,
			CARMEN_FAST_POLAR_SLAM_MEASUREMENT_MODEL_MESSAGE_FMT,
			message, sizeof(carmen_fast_polar_slam_measurement_model_message),
			handler, subscribe_how);
}

void
carmen_fast_polar_slam_publish_measurement_model_message(carmen_fast_polar_slam_measurement_model_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_FAST_POLAR_SLAM_MEASUREMENT_MODEL_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_FAST_POLAR_SLAM_MEASUREMENT_MODEL_MESSAGE_NAME);
}


void
carmen_fast_polar_slam_unsubscribe_measurement_model_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_FAST_POLAR_SLAM_MEASUREMENT_MODEL_MESSAGE_NAME, handler);
}


void
carmen_fast_polar_slam_define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_FAST_POLAR_SLAM_BEST_PARTICLE_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FAST_POLAR_SLAM_BEST_PARTICLE_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FAST_POLAR_SLAM_BEST_PARTICLE_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_FAST_POLAR_SLAM_VELODYNE_PROJECTED_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FAST_POLAR_SLAM_VELODYNE_PROJECTED_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FAST_POLAR_SLAM_VELODYNE_PROJECTED_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_FAST_POLAR_SLAM_PARTICLES_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FAST_POLAR_SLAM_PARTICLES_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FAST_POLAR_SLAM_PARTICLES_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_FAST_POLAR_SLAM_MEASUREMENT_MODEL_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FAST_POLAR_SLAM_MEASUREMENT_MODEL_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FAST_POLAR_SLAM_MEASUREMENT_MODEL_MESSAGE_NAME);
}


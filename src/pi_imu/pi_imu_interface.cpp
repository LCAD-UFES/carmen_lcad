#include <carmen/carmen.h>
#include <carmen/pi_imu_messages.h>
#include <carmen/pi_imu_interface.h>
#include <iostream>

// ==============================================================
// UNSUBSCRIBES
void
carmen_pi_imu_unsubscribe(carmen_handler_t handler)
{
	carmen_unsubscribe_message((char *)CARMEN_PI_IMU_NAME , handler);
}

// ==============================================================
// SUBSCRIBES
void
carmen_pi_imu_subscribe(carmen_pi_imu_message_t *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char *)CARMEN_PI_IMU_NAME, (char *)CARMEN_PI_IMU_FMT,
			message, sizeof(carmen_pi_imu_message_t),
			handler, subscribe_how);
}

// ==============================================================
// PUBLISHES
IPC_RETURN_TYPE
carmen_pi_imu_publish_message(carmen_pi_imu_message_t *message)
{
	IPC_RETURN_TYPE err;
    err = IPC_publishData(CARMEN_PI_IMU_NAME, message);
    carmen_test_ipc_exit(err, "Could not publish!", CARMEN_PI_IMU_NAME);
    return err;
}




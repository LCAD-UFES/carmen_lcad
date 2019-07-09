#include <carmen/carmen.h>
#include <carmen/xsens_messages.h>
#include <math.h>
#include <carmen/pi_imu_interface.h>

carmen_pi_imu_message_t *imu_msg;



void
carmen_publish_xsens_quat_message(carmen_xsens_global_quat_message xsens_quat_message)
{
	xsens_quat_message.timestamp = carmen_get_time();
	xsens_quat_message.host = carmen_get_host();

	// Acceleration
	xsens_quat_message.m_acc.x = imu_msg->imu_vector.accel.x;
	xsens_quat_message.m_acc.y = imu_msg->imu_vector.accel.y;
	xsens_quat_message.m_acc.z = imu_msg->imu_vector.accel.z;
    xsens_quat_message.m_gyr.x = imu_msg->imu_vector.gyro.x;
    xsens_quat_message.m_gyr.y = imu_msg->imu_vector.gyro.y;
    xsens_quat_message.m_gyr.z = imu_msg->imu_vector.gyro.z;
    xsens_quat_message.m_mag.x = imu_msg->imu_vector.magnetometer.x;
    xsens_quat_message.m_mag.y = imu_msg->imu_vector.magnetometer.y;
    xsens_quat_message.m_mag.z = imu_msg->imu_vector.magnetometer.z;
    xsens_quat_message.quat_data.m_data[0] = imu_msg->imu_vector.quat_data.m_data[0];
    xsens_quat_message.quat_data.m_data[1] = imu_msg->imu_vector.quat_data.m_data[1];
    xsens_quat_message.quat_data.m_data[2] = imu_msg->imu_vector.quat_data.m_data[2];
    xsens_quat_message.quat_data.m_data[3] = imu_msg->imu_vector.quat_data.m_data[3];
	xsens_quat_message.m_temp = 0.0;
	xsens_quat_message.m_count = 12345;
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_XSENS_GLOBAL_QUAT_NAME, &xsens_quat_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_GLOBAL_QUAT_NAME);
}

void
imu_handler(carmen_pi_imu_message_t* msg)
{
	imu_msg->imu_vector.accel.x = msg->imu_vector.accel.x;
	imu_msg->imu_vector.accel.y = msg->imu_vector.accel.y;
	imu_msg->imu_vector.accel.z = msg->imu_vector.accel.z;
	imu_msg->imu_vector.magnetometer.x	= msg->imu_vector.magnetometer.x;
	imu_msg->imu_vector.magnetometer.y	= msg->imu_vector.magnetometer.y;
	imu_msg->imu_vector.magnetometer.z	= msg->imu_vector.magnetometer.z;
	imu_msg->imu_vector.gyro.x = msg->imu_vector.gyro.x;
	imu_msg->imu_vector.gyro.y = msg->imu_vector.gyro.y;
	imu_msg->imu_vector.gyro.z = msg->imu_vector.gyro.z;
    imu_msg->imu_vector.quat_data.m_data[0] = msg->imu_vector.quat_data.m_data[0];
    imu_msg->imu_vector.quat_data.m_data[1] = msg->imu_vector.quat_data.m_data[1];
    imu_msg->imu_vector.quat_data.m_data[2] = msg->imu_vector.quat_data.m_data[2];
    imu_msg->imu_vector.quat_data.m_data[3] = msg->imu_vector.quat_data.m_data[3];

	/*printf ("%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
			imu_msg->imu_vector.accel.x, imu_msg->imu_vector.accel.y, imu_msg->imu_vector.accel.z,
			imu_msg->imu_vector.gyro.x, imu_msg->imu_vector.gyro.y, imu_msg->imu_vector.gyro.z,
			imu_msg->imu_vector.magnetometer.x, imu_msg->imu_vector.magnetometer.y, imu_msg->imu_vector.magnetometer.z);*/
}

int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);
    carmen_xsens_global_quat_message xsens_quat_message;
    imu_msg = (carmen_pi_imu_message_t*) calloc(1, sizeof (carmen_pi_imu_message_t));

	carmen_pi_imu_subscribe(NULL, (carmen_handler_t) imu_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_publish_xsens_quat_message(xsens_quat_message);

    carmen_ipc_dispatch();

}



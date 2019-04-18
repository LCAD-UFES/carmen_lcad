/*
 * pi_imu_server_logger.cpp
 *
 *  Created on: 15 de abr de 2019
 *      Author: marcelo
 */
#include <carmen/pi_imu_interface.h>
#include <carmen/pi_imu_messages.h>
#include <carmen/carmen.h>
#include <math.h>

carmen_pi_imu_message_t *pi_imu_msg;

#define m_accel_scale 0.000244;
#define m_gyro_scale 0.0175 * (M_PI / 180.);
#define m_magnetometer_scale 0.014;

void
carmen_xsens_define_messages()
{
    IPC_RETURN_TYPE err;

    /* register xsens's global message */
    err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_QUAT_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_QUAT_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_QUAT_NAME);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Handlers																						//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////

void
pi_imu_message_handler(carmen_pi_imu_message_t *message)
{
	pi_imu_msg->imu_vector->accel.x = message->imu_vector->accel.x;
	pi_imu_msg->imu_vector->accel.y = message->imu_vector->accel.y;
	pi_imu_msg->imu_vector->accel.z = message->imu_vector->accel.z;
	pi_imu_msg->imu_vector->gyro.x = message->imu_vector->gyro.x;
	pi_imu_msg->imu_vector->gyro.y = message->imu_vector->gyro.y;
	pi_imu_msg->imu_vector->gyro.z = message->imu_vector->gyro.z;
	pi_imu_msg->imu_vector->magnetometer.x = message->imu_vector->magnetometer.x;
	pi_imu_msg->imu_vector->magnetometer.y = message->imu_vector->magnetometer.y;
	pi_imu_msg->imu_vector->magnetometer.z = message->imu_vector->magnetometer.z;
}



void
shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();

        printf("Imu Client Logger: disconnected.\n");
        exit(0);
    }
}

void
normalize(carmen_vector_3D_t *vector)
{
    float length = sqrt(vector->x * vector->x + vector->y * vector->y +
            vector->z * vector->z);

    if (length == 0)
        return;

    vector->x /= length;
    vector->y /= length;
    vector->z /= length;
}

void
accelToEuler()
{
	carmen_vector_3D_t rollPitchYaw;

    normalize(&pi_imu_msg->imu_vector->accel);

    rollPitchYaw.x = atan2(pi_imu_msg->imu_vector->accel.y, pi_imu_msg->imu_vector->accel.z);
    rollPitchYaw.y = -atan2(pi_imu_msg->imu_vector->accel.x, sqrt(pi_imu_msg->imu_vector->accel.y
    		* pi_imu_msg->imu_vector->accel.y + pi_imu_msg->imu_vector->accel.z *
			pi_imu_msg->imu_vector->accel.z));
    rollPitchYaw.z = 0;
}

carmen_quaternion_t multiplyQuaternion(carmen_quaternion_t qa, carmen_quaternion_t qb)
{
	carmen_quaternion_t qc;

    qc.q0 = qa.q0 * qb.q0 - qa.q1 * qb.q1 - qa.q2 * qb.q2 - qa.q3 * qb.q3;
    qc.q1 = qa.q0 * qb.q1 + qa.q1 * qb.q0 + qa.q2 * qb.q3 - qa.q3 * qb.q2;
    qc.q2 = qa.q0 * qb.q2 - qa.q1 * qb.q3 + qa.q2 * qb.q0 + qa.q3 * qb.q1;
    qc.q3 = qa.q0 * qb.q3 + qa.q1 * qb.q2 - qa.q2 * qb.q1 + qa.q3 * qb.q0;

    return qc;
}

void
fromEuler(carmen_vector_3D_t& vec)
{
    float cosX2 = cos(vec.x / 2.0f);
    float sinX2 = sin(vec.x / 2.0f);
    float cosY2 = cos(vec.y / 2.0f);
    float sinY2 = sin(vec.y / 2.0f);
    float cosZ2 = cos(vec.z / 2.0f);
    float sinZ2 = sin(vec.z / 2.0f);

    m_data[0] = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
    m_data[1] = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
    m_data[2] = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
    m_data[3] = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;
    normalize();
}

void
calculatePose(const carmen_vector_3D_t& accel, const carmen_vector_3D_t& mag, float magDeclination)
{
    carmen_quaternion_t m;
    carmen_quaternion_t q;

    if (m_enableAccel) {
        accel.accelToEuler(m_measuredPose);
    } else {
        m_measuredPose = m_fusionPose;
        m_measuredPose.setZ(0);
    }

    if (m_enableCompass && m_compassValid) {
        q.fromEuler(m_measuredPose);
        m.setScalar(0);
        m.setX(mag.x());
        m.setY(mag.y());
        m.setZ(mag.z());

        m = q * m * q.conjugate();
        m_measuredPose.setZ(-atan2(m.y(), m.x()) - magDeclination);
    } else {
        m_measuredPose.setZ(m_fusionPose.z());
    }

    m_measuredQPose.fromEuler(m_measuredPose);

    //  check for quaternion aliasing. If the quaternion has the wrong sign
    //  the kalman filter will be very unhappy.

    int maxIndex = -1;
    float maxVal = -1000;

    for (int i = 0; i < 4; i++) {
        if (fabs(m_measuredQPose.data(i)) > maxVal) {
            maxVal = fabs(m_measuredQPose.data(i));
            maxIndex = i;
        }
    }

    //  if the biggest component has a different sign in the measured and kalman poses,
    //  change the sign of the measured pose to match.

    if (((m_measuredQPose.data(maxIndex) < 0) && (m_fusionQPose.data(maxIndex) > 0)) ||
            ((m_measuredQPose.data(maxIndex) > 0) && (m_fusionQPose.data(maxIndex) < 0))) {
        m_measuredQPose.setScalar(-m_measuredQPose.scalar());
        m_measuredQPose.setX(-m_measuredQPose.x());
        m_measuredQPose.setY(-m_measuredQPose.y());
        m_measuredQPose.setZ(-m_measuredQPose.z());
        m_measuredQPose.toEuler(m_measuredPose);
    }
}


void
update_imu_msg_with_true_data(carmen_pi_imu_message_t *msg)
{
	msg->imu_vector->accel.x =  msg->imu_vector->accel.x * m_accel_scale;
	msg->imu_vector->accel.y =  msg->imu_vector->accel.y * m_accel_scale;
	msg->imu_vector->accel.z =  msg->imu_vector->accel.z * m_accel_scale;
	msg->imu_vector->gyro.x =  msg->imu_vector->gyro.x * m_gyro_scale;
	msg->imu_vector->gyro.y =  msg->imu_vector->gyro.y * m_gyro_scale;
	msg->imu_vector->gyro.z =  msg->imu_vector->gyro.z * m_gyro_scale;
	msg->imu_vector->magnetometer.x =  msg->imu_vector->magnetometer.x * m_magnetometer_scale;
	msg->imu_vector->magnetometer.y =  msg->imu_vector->magnetometer.y * m_magnetometer_scale;
	msg->imu_vector->magnetometer.z =  msg->imu_vector->magnetometer.z * m_magnetometer_scale;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Inicializations																				//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char *argv[])
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);

	pi_imu_msg = (carmen_pi_imu_message_t*) calloc (1, sizeof (carmen_pi_imu_message_t));

	carmen_pi_imu_subscribe(NULL,
				  (carmen_handler_t) pi_imu_message_handler,
				  CARMEN_SUBSCRIBE_ALL);

	carmen_xsens_global_quat_message xsens_quat_message;

	return 0;
}

/*********************************************************
 Pi IMU Module
 *********************************************************/

#ifndef CARMEN_PI_IMU_MESSAGES_H
#define CARMEN_PI_IMU_MESSAGES_H

#include <carmen/carmen.h>
#include <carmen/global.h>

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct
{
	carmen_vector_3D_t accel;
	carmen_vector_3D_t gyro;
	carmen_vector_3D_t magnetometer;
} carmen_imu_t;


typedef struct
{
	carmen_imu_t imu_vector;
	double timestamp;
	char *host;
} carmen_pi_imu_message_t;

#define CARMEN_PI_IMU_NAME 	"carmen_pi_imu_message"
#define CARMEN_PI_IMU_FMT 	"{{{double,double,double},{double,double,double},{double,double,double}},double,string}"

#ifdef __cplusplus
}
#endif

#endif

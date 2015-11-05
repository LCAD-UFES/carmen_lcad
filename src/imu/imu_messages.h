#ifndef CARMEN_IMU_MESSAGES_H
#define CARMEN_IMU_MESSAGES_H

/** @addtogroup imu_interface libimu_interface **/
// @{

#ifdef __cplusplus
extern "C" {
#endif

/**
 * information provided by an imu
 */
typedef struct {
  double accX;    ///< acceleration in x direction
  double accY;    ///< acceleration in y direction
  double accZ;    ///< acceleration in z direction
  double q0;      ///< Quaternion describing the rotation, w elem
  double q1;      ///< Quaternion describing the rotation, x elem
  double q2;      ///< Quaternion describing the rotation, y elem
  double q3;      ///< Quaternion describing the rotation, z elem
  double magX;    ///< magnetic field vector
  double magY;    ///< magnetic field vector
  double magZ;    ///< magnetic field vector
  double gyroX;   ///< gyro rotvel in radians/s
  double gyroY;   ///< gyro rotvel in radians/s
  double gyroZ;   ///< gyro rotvel in radians/s
  double timestamp;
  char *host;
} carmen_imu_message;

#define CARMEN_IMU_MESSAGE_NAME "carmen_imu_message"
#define CARMEN_IMU_MESSAGE_FMT  "{double, double, double, double, double, double, double, double, double, double, double, double, double, double, string}"


typedef struct {
  int isalive;
} carmen_imu_alive_message;
  
#define      CARMEN_IMU_ALIVE_NAME            "carmen_imu_alive"
#define      CARMEN_IMU_ALIVE_FMT             "{int}"



#ifdef __cplusplus
}
#endif

// @}

#endif

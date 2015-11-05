/*********************************************************
	 ---   My Module Specific Messages ---

See IPC documentation for more information:
http://www.cs.cmu.edu/~ipc/

*********************************************************/

#ifndef CARMEN_EKF_ODOMETRY_MESSAGES_H
#define CARMEN_EKF_ODOMETRY_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Message Struct Example */
typedef struct {
  carmen_pose_3D_t estimated_pose;
  carmen_pose_3D_t neural_global_pose;
  carmen_vector_3D_t saliencies[5];
  carmen_vector_3D_t correspondences[5];
  double covariance[6][6];
  double timestamp; 		/* !!! obrigatory !!! */
  char *host; 			/* !!! obrigatory !!! */
} carmen_ekf_odometry_odometry_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_EKF_ODOMETRY_ODOMETRY_MESSAGE_NAME       "carmen_ekf_odometry_odometry_message"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_EKF_ODOMETRY_ODOMETRY_MESSAGE_FMT       "{{{double, double, double}, {double, double, double}}, {{double, double, double}, {double, double, double}}, [{double, double, double}:5], [{double, double, double}:5], [double:6,6],  double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}

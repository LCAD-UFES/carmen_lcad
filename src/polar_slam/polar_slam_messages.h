
#ifndef CARMEN_POLAR_SLAM_MESSAGES_H
#define CARMEN_POLAR_SLAM_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	char caracter;
	double timestamp;
	char *host;
} carmen_polar_slam_message;

#define CARMEN_POLAR_SLAM_MESSAGE_NAME "carmen_polar_slam_message"
#define CARMEN_POLAR_SLAM_MESSAGE_FMT "{byte,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}


#ifndef _CARMEN_MOVING_OBJECTS3_MESSAGES_H_
#define _CARMEN_MOVING_OBJECTS3_MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
	int num_rays;
	double *ranges;
	double *angles;
	double *intensity;
	double timestamp;
	char *host;
} carmen_velodyne_projected_on_ground_message;

#define CARMEN_VELODYNE_PROJECTED_MESSAGE_NAME "carmen_fast_polar_slam_velodyne_projected_on_ground_message"
#define CARMEN_VELODYNE_PROJECTED_MESSAGE_FMT "{int,<{double}:1>,<{double}:1>,<{double}:1>,double,string}"


#ifdef __cplusplus
}
#endif

#endif

// @}

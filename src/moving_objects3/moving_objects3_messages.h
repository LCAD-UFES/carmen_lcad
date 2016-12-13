
#ifndef _CARMEN_MOVING_OBJECTS3_MESSAGES_H_
#define _CARMEN_MOVING_OBJECTS3_MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/moving_objects3_particle_filter.h>

typedef struct
{
	int num_rays;
	double *ranges;
	double *angles;
	double *intensity;
	double timestamp;
	char *host;
} carmen_velodyne_projected_on_ground_message;

#define CARMEN_VELODYNE_PROJECTED_MESSAGE_NAME "carmen_velodyne_projected_on_ground_message"
#define CARMEN_VELODYNE_PROJECTED_MESSAGE_FMT "{int,<{double}:1>,<{double}:1>,<{double}:1>,double,string}"


typedef struct
{
	int num_particles;
	moving_objects3_particle_t *particles;
	double timestamp;
	char *host;
} carmen_moving_objects3_particles_message;

#define CARMEN_MOVING_OBJECTS3_PARTICLES_MESSAGE_NAME "carmen_moving_objects3_particles_message"
#define CARMEN_MOVING_OBJECTS3_PARTICLES_MESSAGE_FMT "{int,<{{double,double,double},double,{double,double,double,double},double}:1>,double,string}"


#ifdef __cplusplus
}
#endif

#endif

// @}

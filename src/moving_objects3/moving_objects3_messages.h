#ifndef _CARMEN_MOVING_OBJECTS3_MESSAGES_H_
#define _CARMEN_MOVING_OBJECTS3_MESSAGES_H_


typedef struct
{
	double width;	// object width
	double length;	// object length
	double c_x;		// anchor point x
	double c_y;		// anchor point y
} geometric_parameters;


typedef struct
{
	carmen_point_t pose;	// object pose (x, y, theta)
	double velocity;		// object velocity
	geometric_parameters geometry; // geomeric parameters
	double weight;			// particle weight
} moving_objects3_particle_t;


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


#endif

// @}

#ifndef CARMEN_SLAM_MESSAGES_H
#define CARMEN_SLAM_MESSAGES_H

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialization message for localize
#define CARMEN_SLAM_INITIALIZE_UNIFORM	1
#define CARMEN_SLAM_INITIALIZE_GAUSSIAN	2

	typedef struct {
		carmen_point_t globalpos, globalpos_std;
		carmen_point_t odometrypos;
		double globalpos_xy_cov;
		double timestamp;
		char *host;
	} carmen_slam_globalpos_message;

#define CARMEN_SLAM_GLOBALPOS_FMT  "{{double,double,double},{double,double,double},{double,double,double},double,double,string}"
#define CARMEN_SLAM_GLOBALPOS_NAME  "carmen_slam_globalpos_msg"

	typedef struct {
		float x, y, theta, weight;
	} carmen_slam_particles_t, *carmen_slam_particles_p;

	typedef struct {
		int num_particles;
		carmen_slam_particles_p particles;
		carmen_point_t globalpos;
		double timestamp;
		char *host;
	} carmen_slam_particles_message;

#define CARMEN_SLAM_PARTICLES_FMT  "{int,<{float,float,float,float}:1>,{double,double,double},double,string}"
#define CARMEN_SLAM_PARTICLES_NAME  "carmen_slam_particles_msg"

	typedef struct {
	  carmen_laser_laser_config_t config;
	  int num_readings, laser_skip;
	  float *range;
	  carmen_point_t pose;
	  int num_laser;
	  double timestamp;
	  char *host;
	} carmen_slam_laser_message;

	#define CARMEN_SLAM_LASER_NAME "carmen_slam_laser_msg"
	#define CARMEN_SLAM_LASER_FMT  "{{int,double,double,double,double,double,int},int,int,<float:2>,{double,double,double},int,double,string}"

	typedef struct {
		  int distribution_type;
		  int distribution_modes;
		  carmen_point_t *mean, *std;
		  double timestamp;
		  char *host;
	} carmen_slam_initialize_message;
	#define CARMEN_SLAM_INITIALIZE_NAME "carmen_slam_initialize_msg"
	#define CARMEN_SLAM_INITIALIZE_FMT  "{int,int,<{double,double,double}:2>,<{double,double,double}:2>,double,string}"

*/

#ifdef __cplusplus
}
#endif

#endif

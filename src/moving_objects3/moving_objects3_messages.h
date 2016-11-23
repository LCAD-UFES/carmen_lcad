
#ifndef _CARMEN_FAST_POLAR_SLAM_MESSAGES_H_
#define _CARMEN_FAST_POLAR_SLAM_MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/fast_polar_particle_filter.h>


typedef struct
{
	int map_size;
	carmen_pose_3D_t particle_pose;
	carmen_pose_3D_t pose_before_movement;
	int *particle_map_occupation;
	carmen_polar_point_t *particle_map;
	double weight;
	double timestamp;
	char *host;
} carmen_fast_polar_slam_best_particle_message;

#define CARMEN_FAST_POLAR_SLAM_BEST_PARTICLE_MESSAGE_NAME "carmen_fast_polar_slam_best_particle_message"
#define CARMEN_FAST_POLAR_SLAM_BEST_PARTICLE_MESSAGE_FMT "{int,{{double,double,double},{double,double,double}},{{double,double,double},{double,double,double}},<{int}:1>,<{double,double}:1>,double,double,string}"


typedef struct
{
	int num_particles;
	carmen_pose_3D_t *particle_pose;
	double *weight;
	double timestamp;
	char *host;
} carmen_fast_polar_slam_particles_message;

#define CARMEN_FAST_POLAR_SLAM_PARTICLES_MESSAGE_NAME "carmen_fast_polar_slam_particles_message"
#define CARMEN_FAST_POLAR_SLAM_PARTICLES_MESSAGE_FMT "{int,<{{double,double,double},{double,double,double}}:1>,<{double}:1>,double,string}"


typedef struct
{
	int num_rays;
	double *ranges;
	double *angles;
	double *intensity;
	double timestamp;
	char *host;
} carmen_fast_polar_slam_velodyne_projected_on_ground_message;

#define CARMEN_FAST_POLAR_SLAM_VELODYNE_PROJECTED_MESSAGE_NAME "carmen_fast_polar_slam_velodyne_projected_on_ground_message"
#define CARMEN_FAST_POLAR_SLAM_VELODYNE_PROJECTED_MESSAGE_FMT "{int,<{double}:1>,<{double}:1>,<{double}:1>,double,string}"


typedef struct
{
	int num_rays;
	double *angles;
	double *measured_ranges;
	double *estimated_ranges;
	double *ray_probability;
	double timestamp;
	char *host;
}carmen_fast_polar_slam_measurement_model_message;

#define CARMEN_FAST_POLAR_SLAM_MEASUREMENT_MODEL_MESSAGE_NAME "carmen_fast_polar_slam_measurement_model_message"
#define CARMEN_FAST_POLAR_SLAM_MEASUREMENT_MODEL_MESSAGE_FMT "{int,<{double}:1>,<{double}:1>,<{double}:1>,<{double}:1>,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}

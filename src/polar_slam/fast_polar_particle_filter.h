/**
 * @file
 * @author Filipe Mutz
 *
 * @section DESCRIPTION
 * This file defines the structure of the polar map and the particle filter in a more efficient way
 */

#ifndef FAST_POLAR_PARTICLE_FILTER_H_
#define FAST_POLAR_PARTICLE_FILTER_H_

#include <carmen/carmen.h>
#include <carmen/fused_odometry_interface.h>
#include <prob_motion_model.h>
#include <prob_measurement_model.h>
#include <carmen/polar_point.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
	int map_size;
	int num_particles;
	int num_spheres;
	int num_sections;
	int num_points_per_sections;
	int first_map_has_been_received;
	double max_range;

}carmen_polar_particle_filter_config;


typedef struct
{
	carmen_pose_3D_t particle_pose;
	carmen_pose_3D_t pose_before_movement;
	int *particle_map_occupation;
	carmen_polar_point_t *particle_map;
	double weight;

}carmen_polar_particle;


typedef struct
{
	carmen_polar_particle_filter_config config;
	carmen_polar_particle *particles;
	carmen_polar_particle *best_particle;

}carmen_polar_particle_filter;


/**
 * This function allocate a particle filter and initialize its pose with the global pose param
 *
 * @param carmen_pose_3D_t global_pose this pose is used as the mean pose of the particles
 * @param int num_particles the number of particles in the particle filter
 * @param int num_spheres the number of spheres in each particle's map
 * @param int num_sections the number of angular sections in each sphere (it can be calculate using the desired angular resolution in the following formula: num_sections = (360 * angular_resolution)
 * @param int num_points_per_section the number of points to be stored in each section of the sphere. the points are stored in ascending order of its radius. In that way, increase the number of points causes a richer vision of surroundings.
 * @return carmen_polar_particle_filter* a pointer to the particle filter created
 */
carmen_polar_particle_filter* carmen_polar_slam_create_particle_filter(carmen_pose_3D_t *global_pose, int num_particles, int num_spheres, int num_sections, int num_points_per_section, double max_range);


/**
 * T
 *
 * @param
 * @param
 * @param
 * @return
 */
void carmen_polar_slam_correct_pose_using_beam_range_finder_model(carmen_polar_particle_filter* particle_filter, double *range, double *angles, int num_rays);


/**
 * T
 *
 * @param
 * @param
 * @param
 * @return
 */
void carmen_polar_slam_add_obstacles_to_map(carmen_polar_particle_filter* particle_filter, double *ranges, double *angles, int num_rays);


/**
 * T
 *
 * @param
 * @param
 * @param
 * @return
 */
void carmen_polar_slam_predict_pose_using_motion_model(carmen_polar_particle_filter* particle_filter, OdometryMotionCommand *ut, carmen_fused_odometry_message *odometry);


/**
 * T
 *
 * @param
 * @param
 * @param
 * @return
 */

int carmen_polar_slam_get_point_position(carmen_polar_particle_filter *particle_filter, int sphere_id, int section_id);


/**
 * T
 *
 * @param
 * @param
 * @param
 * @return
 */
void carmen_polar_slam_move_particles_to_new_position(carmen_polar_particle_filter* particle_filter);


/**
 * T
 *
 * @param
 * @param
 * @param
 * @return
 */
double carmen_polar_slam_perform_ray_cast(carmen_polar_particle_filter* particle_filter, int particle, double angle, double measured_range);

#ifdef __cplusplus
}
#endif
#endif



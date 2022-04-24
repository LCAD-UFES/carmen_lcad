#ifndef CARMEN_PROB_MONTE_CARLO_H
#define CARMEN_PROB_MONTE_CARLO_H

#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/localize_ackerman_core.h>
#include "prob_measurement_model.h"
#include "prob_motion_model.h"
#include "prob_map.h"

#ifdef __cplusplus
extern "C" 
{
#endif

void 
init_particles_from_gaussians(carmen_point_t *particles, carmen_point_t *_particles, double *weight,
		carmen_point_t *mean,
		carmen_point_t *std,
		int number_of_distribution_modes,
		int number_of_particles);

void
build_globalpos_ackerman_message(carmen_localize_ackerman_globalpos_message *globalpos_ackerman_message,
		const carmen_point_t *particles, const double *weights, const int num_particles,
		carmen_point_t odometry_pose, double v, double phi, int converged, double timestamp);
		
void
build_sensor_ackerman_message(carmen_localize_ackerman_sensor_message *sensor_ackerman_message, const carmen_point_t *robot_pose, const carmen_robot_ackerman_laser_message *laser, double timestamp);

void
build_sensor_ackerman_message2(carmen_localize_ackerman_sensor_message *sensor_ackerman_message,const carmen_point_t *robot_pose,
		float *range, int num_readings, carmen_laser_laser_config_t laser_config, double timestamp);

void
build_particles_ackerman_message(carmen_localize_ackerman_particle_message *particles_ackerman_message,
		const carmen_localize_ackerman_globalpos_message *globalpos_ackerman_message,
		const int num_particles, const carmen_point_t *particles, const double *weights, double timestamp);

carmen_point_t
improved_proposal_distribution_odometry(const OdometryMotionCommand *ut, carmen_point_t xt_1,
		carmen_map_t* map, double *zt,
		int number_of_particles);

carmen_point_t
improved_proposal_distribution_ackerman(const AckermanMotionCommand *ut, carmen_point_t xt_1,
		carmen_map_t* map, double *zt,
		int number_of_particles);

carmen_point_t
improved_proposal_distribution_velocity(const VelocityMotionCommand *ut, carmen_point_t xt_1,
		carmen_map_t* map, double *zt,
		int number_of_particles);

#ifdef __cplusplus
}
#endif

#endif

#ifndef LOCALIZE_ACKERMAN_TRAILERS_THETA_H
#define LOCALIZE_ACKERMAN_TRAILERS_THETA_H

#include <carmen/carmen.h>


#ifdef __cplusplus
extern "C" {
#endif

double
compute_semi_trailer_theta1(carmen_robot_and_trailers_traj_point_t robot_and_trailer_traj_point, double dt, double v,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailers_config_t semi_trailer_config, 
        sensor_parameters_t *spherical_sensor_params, 
        void *message, int lidar_to_compute_theta);

#ifdef __cplusplus
}
#endif

#endif

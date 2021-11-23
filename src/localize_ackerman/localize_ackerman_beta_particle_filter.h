#ifndef CARMEN_LOCALIZE_ACKERMAN_BETA_PARTICLE_FILTER_H
#define CARMEN_LOCALIZE_ACKERMAN_BETA_PARTICLE_FILTER_H

#include <carmen/carmen.h>
#include <carmen/car_model.h>

#ifdef __cplusplus
extern "C" {
#endif


void
carmen_localize_ackerman_beta_prediction(carmen_localize_ackerman_particle_filter_p filter, carmen_robot_and_trailer_traj_point_t robot_and_trailer_traj_point,
										 carmen_robot_ackerman_config_t robot_config, carmen_semi_trailer_config_t semi_trailer_config);

void
carmen_localize_ackerman_beta_correction();

void
carmen_localize_ackerman_beta_resample();


#ifdef __cplusplus
}
#endif

#endif
// @}

#ifndef CARMEN_LOCALIZE_ACKERMAN_BETA_PARTICLE_FILTER_H
#define CARMEN_LOCALIZE_ACKERMAN_BETA_PARTICLE_FILTER_H

#include <carmen/carmen.h>
#include <carmen/car_model.h>

#ifdef __cplusplus
extern "C" {
#endif


void
carmen_localize_ackerman_beta_prediction(carmen_localize_ackerman_particle_filter_p filter, carmen_robot_and_trailer_traj_point_t robot_and_trailer_traj_point,
										 carmen_robot_ackerman_config_t robot_config, carmen_semi_trailer_config_t semi_trailer_config, double velodyne_timestamp);

void
carmen_localize_ackerman_beta_correction(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_map_p localize_map, carmen_compact_map_t *local_map, carmen_compact_map_t *local_mean_remission_map,
		carmen_compact_map_t *local_variance_remission_map  __attribute__ ((unused)),
		carmen_localize_ackerman_binary_map_t *binary_map __attribute__ ((unused)));


void
carmen_localize_ackerman_summarize_beta(carmen_localize_ackerman_particle_filter_p filter, carmen_localize_ackerman_summary_p summary);


void
carmen_localize_ackerman_beta_resample(carmen_localize_ackerman_particle_filter_p filter);


#ifdef __cplusplus
}
#endif

#endif
// @}

#include "localize_ackerman_beta_particle_filter.h"



void
carmen_localize_ackerman_beta_prediction(carmen_localize_ackerman_particle_filter_p filter, carmen_robot_and_trailer_traj_point_t robot_and_trailer_traj_point,
										 carmen_robot_ackerman_config_t robot_config, carmen_semi_trailer_config_t semi_trailer_config)
{
	double dt = 0.001;
	compute_semi_trailer_beta(robot_and_trailer_traj_point, dt, robot_config, semi_trailer_config);
}


void
carmen_localize_ackerman_beta_correction()
{

}


void
carmen_localize_ackerman_beta_resample()
{

}

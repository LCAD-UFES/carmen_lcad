#include "localize_ackerman_beta_particle_filter.h"



void
carmen_localize_ackerman_beta_prediction(carmen_localize_ackerman_particle_filter_p filter, carmen_robot_and_trailers_traj_point_t robot_and_trailer_traj_point,
										 carmen_robot_ackerman_config_t robot_config, carmen_semi_trailer_config_t semi_trailer_config, double dt)
{
	if (!filter->initialized)
			return;

	double v_step, phi_step, v, phi;
	carmen_robot_and_trailers_traj_point_t robot_pose;

	if (fabs(dt) > 3.0) // Possivelmente reposicionamento do robo na interface
		return;

	v = robot_and_trailer_traj_point.v;
	phi = robot_and_trailer_traj_point.phi;
	filter->distance_travelled += fabs(v * dt);

	for (int i = 0; i < filter->param->num_particles; i++)
	{
		robot_pose = {robot_and_trailer_traj_point.x ,robot_and_trailer_traj_point.y, robot_and_trailer_traj_point.theta, robot_and_trailer_traj_point.trailer_theta[0], robot_and_trailer_traj_point.v, robot_and_trailer_traj_point.phi};

		if (i != 0)
		{
			v_step = v + carmen_gaussian_random(0.0,
					fabs(filter->param->velocity_noise_velocity * v) +
					fabs(filter->param->velocity_noise_phi * phi) + filter->param->v_uncertainty_at_zero_v);

			if (fabs(v) > 0.05)
			{
				// This update is similar to one used in "Map-Based Precision Vehicle Localization in Urban Environments" for updating the gps bias.
				// In this paper, however, an additional factor is used for slowly pulling the bias towards zero.
				// Using the rule from the paper, the update would be: phi = phi * gamma + N(0, phi_bias_std), where gamma = 0.9999.
				filter->particles[i].phi_bias += carmen_gaussian_random(0.0, filter->param->phi_bias_std);

				// 0.0175 radians is approx 1 degree
				filter->particles[i].phi_bias = carmen_clamp(-0.0175 / 2.0, filter->particles[i].phi_bias, 0.0175 / 2.0);
			}
			phi_step = phi + filter->particles[i].phi_bias + carmen_gaussian_random(0.0,
					fabs(filter->param->phi_noise_phi * phi) +
					fabs(filter->param->phi_noise_velocity * v));
			phi_step = carmen_clamp(-M_PI/4.0, phi_step, M_PI/4.0);

			robot_pose.v = v_step;
			robot_pose.phi = phi_step;

			filter->particles[i].theta = compute_semi_trailer_beta(robot_and_trailer_traj_point, dt, robot_config, semi_trailer_config);
			filter->particles[i].phi = phi_step;
			filter->particles[i].v = v_step;
		}

		else
		{
			// Keep the mean particle of the previous run intact
			// Note: This kind of elitism may introduce a bias in the filter.

			v_step = v;
			phi_step = phi + filter->particles[i].phi_bias;

			robot_pose.v = v_step;
			robot_pose.phi = phi_step;

			filter->particles[i].theta = compute_semi_trailer_beta(robot_and_trailer_traj_point, dt, robot_config, semi_trailer_config);
			filter->particles[i].phi = phi_step;
			filter->particles[i].v = v_step;
		}
	}
}


void
carmen_localize_ackerman_beta_correction(carmen_localize_ackerman_particle_filter_p filter __attribute__ ((unused)), carmen_localize_ackerman_map_p localize_map __attribute__ ((unused)),
		carmen_compact_map_t *local_map __attribute__ ((unused)), carmen_compact_map_t *local_mean_remission_map __attribute__ ((unused)),
		carmen_compact_map_t *local_variance_remission_map  __attribute__ ((unused)),
		carmen_localize_ackerman_binary_map_t *binary_map __attribute__ ((unused)))
{

}


void
carmen_localize_ackerman_summarize_beta(carmen_localize_ackerman_particle_filter_p filter __attribute__ ((unused)), carmen_localize_ackerman_summary_p summary __attribute__ ((unused)))
{

}


void
carmen_localize_ackerman_beta_resample(carmen_localize_ackerman_particle_filter_p filter)
{
	/* check if it is time to resample  (verificar se é necessário) */
	if (filter->distance_travelled >= filter->param->update_distance)
	{
		carmen_localize_ackerman_resample(filter);
		filter->distance_travelled = 0.0;
		filter->initialized = 1;
	}
}



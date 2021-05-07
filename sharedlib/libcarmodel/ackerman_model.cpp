#include "car_model.h"


double
compute_semi_trailer_beta(carmen_robot_and_trailer_traj_point_t robot_and_trailer_traj_point, double dt,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailer_config_t semi_trailer_config)
{
	double L = robot_config.distance_between_front_and_rear_axles;
	double beta = robot_and_trailer_traj_point.beta + dt *
			robot_and_trailer_traj_point.v * (tan(robot_and_trailer_traj_point.phi) / L -
						   sin(robot_and_trailer_traj_point.beta) / semi_trailer_config.d +
						   (semi_trailer_config.M / (L * semi_trailer_config.d)) * cos(robot_and_trailer_traj_point.beta) * tan(robot_and_trailer_traj_point.phi));

	return (beta);
}


double
predict_next_pose_step(carmen_robot_and_trailer_traj_point_t *new_robot_state, double target_v, double a, double delta_t,
		double &achieved_curvature, const double &desired_curvature, double &max_curvature_change,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailer_config_t semi_trailer_config)
{
	carmen_robot_and_trailer_traj_point_t initial_robot_state = *new_robot_state;

	double delta_curvature = fabs(achieved_curvature - desired_curvature);
	double command_curvature_signal = (achieved_curvature < desired_curvature) ? 1.0 : -1.0;

	achieved_curvature = achieved_curvature + command_curvature_signal * fmin(delta_curvature, max_curvature_change);
	new_robot_state->phi = carmen_get_phi_from_curvature(achieved_curvature, initial_robot_state.v,
			robot_config.understeer_coeficient,	robot_config.distance_between_front_and_rear_axles);

	new_robot_state->phi = carmen_clamp(-robot_config.max_phi, new_robot_state->phi, robot_config.max_phi);

	double v0 = initial_robot_state.v;
	double s = v0 * delta_t + 0.5 * a * delta_t * delta_t;

	double move_x = s * cos(initial_robot_state.theta);
	double move_y = s * sin(initial_robot_state.theta);

	new_robot_state->x	   += move_x;
	new_robot_state->y	   += move_y;
	new_robot_state->theta += s * tan(new_robot_state->phi) / robot_config.distance_between_front_and_rear_axles;
	new_robot_state->beta = compute_semi_trailer_beta(*new_robot_state, delta_t, robot_config, semi_trailer_config);

	new_robot_state->v = target_v;

	return sqrt(move_x * move_x + move_y * move_y);
}


carmen_robot_and_trailer_traj_point_t
carmen_libcarmodel_recalc_pos_ackerman(carmen_robot_and_trailer_traj_point_t robot_state, double target_v, double target_phi,
		double full_time_interval, double *distance_traveled, double delta_t,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailer_config_t semi_trailer_config)
{
	double a = (target_v - robot_state.v) / full_time_interval;
	int n = floor(full_time_interval / delta_t);
	double remaining_time = full_time_interval - ((double) n * delta_t);
	carmen_robot_and_trailer_traj_point_t achieved_robot_state = robot_state; // achieved_robot_state eh computado iterativamente abaixo a partir do estado atual do robo

	double curvature = carmen_get_curvature_from_phi(target_phi, target_v,
			robot_config.understeer_coeficient,	robot_config.distance_between_front_and_rear_axles);

	double new_curvature = carmen_get_curvature_from_phi(achieved_robot_state.phi, achieved_robot_state.v,
			robot_config.understeer_coeficient,	robot_config.distance_between_front_and_rear_axles);

	double max_curvature_change = robot_config.maximum_steering_command_rate * delta_t;

	// Euler method
	for (int i = 0; i < n; i++)
	{
		double dist_walked = predict_next_pose_step(&achieved_robot_state, target_v, a, delta_t,
				new_curvature, curvature, max_curvature_change, robot_config, semi_trailer_config);

		if (distance_traveled)
			*distance_traveled += dist_walked;
	}

	if (remaining_time > 0.0)
	{
		double dist_walked = predict_next_pose_step(&achieved_robot_state, target_v, a, delta_t,
				new_curvature, curvature, max_curvature_change, robot_config, semi_trailer_config);

		if (distance_traveled)
			*distance_traveled += dist_walked;
	}

	achieved_robot_state.theta = carmen_normalize_theta(achieved_robot_state.theta);
	robot_state = achieved_robot_state;

	return (achieved_robot_state);
}

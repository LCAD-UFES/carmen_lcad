#include "car_model.h"


double
compute_semi_trailer_beta(carmen_robot_and_trailer_traj_point_t robot_and_trailer_traj_point, double dt,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailer_config_t semi_trailer_config)
{
	if (semi_trailer_config.type == 0)
		return (0.0);

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
	delta_t = delta_t / 10.0;
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


#define OLD_STEERING_CONTROL

class Pose
{
public:

	double x;
	double y;
	double theta;
	double beta;
};


class Command
{
public:

	double v;
	double phi;
};

class Robot_State
{
public:

	Pose	pose;
	Command v_and_phi;
};

int signal(double num)
{
	return num >= 0 ? 1 : -1;
}


double
get_acceleration_magnitude(const double &v, const double &desired_v,
		carmen_robot_ackerman_config_t robot_config)
{
	double acceleration = 0.0;

	if (fabs(desired_v) > fabs(v))
	{
		acceleration = robot_config.desired_acceleration;
	}
	else if (fabs(desired_v) < fabs(v))
	{
		acceleration = (desired_v >= 0) ? robot_config.desired_decelaration_forward : robot_config.desired_decelaration_reverse;
	}

	return acceleration;
}


double
get_max_v_change(const double &current_v, const double &desired_v, const double &time,
		carmen_robot_ackerman_config_t robot_config)
{
	double time_spend;
	double v_change;
	double acceleration;
	int signal_current_v, signal_desired_v;

	signal_current_v = signal(current_v);
	signal_desired_v = signal(desired_v);

	if (signal_current_v == signal_desired_v ||
	    fabs(current_v) < 0.001 || fabs(desired_v) < 0.001)
	{
		acceleration = get_acceleration_magnitude(current_v, desired_v, robot_config);

		v_change = fabs(acceleration * time);
	}
	else
	{
		acceleration = get_acceleration_magnitude(current_v, 0, robot_config);
		//desacelerar até 0
		v_change = fmin(fabs(current_v), acceleration * time);

		time_spend = fabs(v_change / acceleration);

		if ((time - time_spend) > 0)
		{
			acceleration = get_acceleration_magnitude(0, desired_v, robot_config);
			v_change += fabs(acceleration * (time - time_spend));
		}
	}

	return v_change;
}


#ifdef OLD_STEERING_CONTROL
double
predict_next_pose_during_main_rrt_planning_step(Robot_State &new_robot_state, const Command &requested_command, double delta_t,
		double &achieved_curvature, const double &desired_curvature, double &max_curvature_change,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailer_config_t semi_trailer_config)
#else
double
predict_next_pose_during_main_rrt_planning_step(Robot_State &new_robot_state, const Command &requested_command, double &phi_velocity,
		const double t, const double delta_t,
		double t_fim_descida, double t_fim_plato, double t_fim_subida,
		double max_phi_velocity, double max_phi_acceleration,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailer_config_t semi_trailer_config)
#endif
{
	Robot_State initial_robot_state = new_robot_state;

#ifdef OLD_STEERING_CONTROL
	double delta_curvature = fabs(achieved_curvature - desired_curvature);
	double command_curvature_signal = (achieved_curvature < desired_curvature) ? 1.0 : -1.0;

	achieved_curvature = achieved_curvature + command_curvature_signal * fmin(delta_curvature, max_curvature_change);
	new_robot_state.v_and_phi.phi = carmen_get_phi_from_curvature(
			achieved_curvature, initial_robot_state.v_and_phi.v,
			robot_config.understeer_coeficient,
			robot_config.distance_between_front_and_rear_axles);
#else
	// Tem que checar se as equacoes que governam esta mudancca de phi estao corretas (precisa de um Euler?) e fazer o mesmo no caso do rrt_path_follower.
	double max_phi_change = get_max_phi_change(initial_robot_state.v_and_phi.phi, requested_command.phi, phi_velocity,
			t, delta_t,
			t_fim_descida, t_fim_plato, t_fim_subida,
			max_phi_velocity, max_phi_acceleration);
	new_robot_state.v_and_phi.phi = initial_robot_state.v_and_phi.phi + max_phi_change;
#endif

	new_robot_state.v_and_phi.phi = carmen_clamp(-robot_config.max_phi, new_robot_state.v_and_phi.phi, robot_config.max_phi);

	// Tem que checar se as equacoes que governam esta mudancca de v estao corretas (precisa de um Euler?) e fazer o mesmo no caso do rrt_path_follower.
	double max_v_change = get_max_v_change(initial_robot_state.v_and_phi.v, requested_command.v, delta_t,robot_config);
	double delta_v = fabs(initial_robot_state.v_and_phi.v - requested_command.v);
	double command_v_signal = (initial_robot_state.v_and_phi.v < requested_command.v) ? 1.0 : -1.0;
	new_robot_state.v_and_phi.v = initial_robot_state.v_and_phi.v + command_v_signal * fmin(delta_v, max_v_change);

	double move_x = initial_robot_state.v_and_phi.v * delta_t * cos(initial_robot_state.pose.theta);
	double move_y = initial_robot_state.v_and_phi.v * delta_t * sin(initial_robot_state.pose.theta);

	new_robot_state.pose.x	   += move_x;
	new_robot_state.pose.y	   += move_y;
	new_robot_state.pose.theta += initial_robot_state.v_and_phi.v * delta_t * tan(initial_robot_state.v_and_phi.phi) / robot_config.distance_between_front_and_rear_axles;

	carmen_robot_and_trailer_traj_point_t robot_and_trailer_traj_point = {
			new_robot_state.pose.x, new_robot_state.pose.y, new_robot_state.pose.theta, new_robot_state.pose.beta,
			new_robot_state.v_and_phi.v, new_robot_state.v_and_phi.phi
	};
	new_robot_state.pose.beta  = compute_semi_trailer_beta(robot_and_trailer_traj_point, delta_t,
			robot_config, semi_trailer_config);

	return sqrt(move_x * move_x + move_y * move_y);
}


Robot_State
predict_next_pose_during_main_rrt_planning_(const Robot_State &robot_state, const Command &requested_command,
		double full_time_interval, double *distance_traveled, double delta_t,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailer_config_t semi_trailer_config)
{
	if (distance_traveled)
		*distance_traveled = 0.0;

	int n = floor(full_time_interval / delta_t);
	double remaining_time = full_time_interval - ((double) n * delta_t);
	Robot_State achieved_robot_state = robot_state; // achieved_robot_state eh computado iterativamente abaixo a partir do estado atual do robo

#ifdef OLD_STEERING_CONTROL
	double curvature = carmen_get_curvature_from_phi(
			requested_command.phi, requested_command.v,
			robot_config.understeer_coeficient,
			robot_config.distance_between_front_and_rear_axles);

	double new_curvature = carmen_get_curvature_from_phi(
			achieved_robot_state.v_and_phi.phi, achieved_robot_state.v_and_phi.v,
			robot_config.understeer_coeficient,
			robot_config.distance_between_front_and_rear_axles);

	double max_curvature_change = robot_config.desired_steering_command_rate * delta_t;
#else
	double max_phi_velocity = max_phi_velocity;
	double max_phi_acceleration = max_phi_acceleration;

	double phi_velocity = 0.0;
	// O codigo abaixo assume phi_velocity == 0.0 no início do full_time_interval
	double t_fim_descida;
	double t_fim_plato;
	double t_fim_subida;
	compute_intermediate_times(t_fim_subida, t_fim_plato, t_fim_descida, max_phi_acceleration, robot_state.v_and_phi.phi, requested_command.phi,
			full_time_interval,	max_phi_velocity);
#endif

	// Euler method
	for (int i = 0; i < n; i++)
	{
#ifdef OLD_STEERING_CONTROL
		double dist_walked = predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, delta_t,
				new_curvature, curvature, max_curvature_change,
				robot_config, semi_trailer_config);
#else
		double t = (double) i * delta_t;
		double dist_walked = predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, phi_velocity,
				t, delta_t,
				t_fim_descida, t_fim_plato, t_fim_subida,
				max_phi_velocity, max_phi_acceleration,
				robot_config, semi_trailer_config);
#endif

		if (distance_traveled)
			*distance_traveled += dist_walked;
	}

	if (remaining_time > 0.0)
	{
#ifdef OLD_STEERING_CONTROL
		double dist_walked = predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, delta_t,
				new_curvature, curvature, max_curvature_change,
				robot_config, semi_trailer_config);
#else
		double t = (double) n * delta_t;
		double dist_walked = predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, phi_velocity,
				t, remaining_time,
				t_fim_descida, t_fim_plato, t_fim_subida,
				max_phi_velocity, max_phi_acceleration,
				robot_config, semi_trailer_config);
#endif

		if (distance_traveled)
			*distance_traveled += dist_walked;
	}

	achieved_robot_state.pose.theta = carmen_normalize_theta(achieved_robot_state.pose.theta);

	return achieved_robot_state;
}


carmen_robot_and_trailer_traj_point_t
carmen_libcarmodel_recalc_pos_ackerman_new(carmen_robot_and_trailer_traj_point_t robot_state, double target_v, double target_phi,
		double full_time_interval, double *distance_traveled, double delta_t,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailer_config_t semi_trailer_config)
{
	Robot_State robot_state_;
	robot_state_.pose.x = robot_state.x;
	robot_state_.pose.y = robot_state.y;
	robot_state_.pose.theta = robot_state.theta;
	robot_state_.pose.beta = robot_state.beta;
	robot_state_.v_and_phi.v = target_v;
	robot_state_.v_and_phi.phi = target_phi;

	Command requested_command;
	requested_command.v = target_v;
	requested_command.phi = target_phi;

	robot_state_ = predict_next_pose_during_main_rrt_planning_(robot_state_, requested_command,
			full_time_interval, distance_traveled, delta_t,
			robot_config, semi_trailer_config);

	robot_state.x = robot_state_.pose.x;
	robot_state.y = robot_state_.pose.y;
	robot_state.theta = robot_state_.pose.theta;
	robot_state.beta = robot_state_.pose.beta;
	robot_state.v = robot_state_.v_and_phi.v;
	robot_state.phi = robot_state_.v_and_phi.phi;

	return (robot_state);
}



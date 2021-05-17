/*
 * ackerman_util.cpp
 *
 *  Created on: 28/05/2012
 *      Author: romulo
 */

#include "ackerman.h"
#include <float.h>
#include <math.h>
#include "util.h"
#include "obstacle_detection.h"
#include "publisher_util.h"
#include "../model/global_state.h"
#include <algorithm>
#include <carmen/map_server_interface.h>
#include "../rs.h"
#include <carmen/navigator_gui_interface.h>
#include <carmen/car_model.h>


typedef struct {
	Command command;
	double command_time;
	double total_cost;
} Command_and_Cost;


class Compare_Command_Cost
{
public:
	bool operator()(const Command_and_Cost &a, const Command_and_Cost &b) const
	{
		return a.total_cost > b.total_cost;
	}
};

typedef priority_queue<Command_and_Cost, vector<Command_and_Cost>, Compare_Command_Cost> Command_and_Cost_List;

class command_increment {
public:
	double curvature;
	double max_value;
	int max_i;

	double get_increment_value(int i) const
	{
		double x = (double) i / (double) max_i;
		return  max_value * (exp(curvature * x) - 1.0) / (exp(curvature) - 1.0);
	}
};

command_increment
get_command_increment(double curvature, double max_value, int max_i = 36)
{
	command_increment ci;

	ci.curvature = curvature;
	ci.max_value = max_value;
	ci.max_i = max_i;

	return ci;
}

vector<double> phi_mask_vector;

vector<double> Ackerman::velocity_search_vector;
vector<double> Ackerman::minimal_velocity_search_vector;

static double
get_phi_increment(double phi)
{
	if (phi < carmen_degrees_to_radians(0.6))
		return carmen_degrees_to_radians(0.1);

	else if (phi < carmen_degrees_to_radians(1.0))
		return carmen_degrees_to_radians(0.2);

	else if (phi < carmen_degrees_to_radians(2.0))
		return carmen_degrees_to_radians(0.5);

	else if (phi < carmen_degrees_to_radians(10))
		return carmen_degrees_to_radians(2.0);

	else
		return carmen_degrees_to_radians(5.0);
}


void 
Ackerman::init_search_parameters()
{
	double	initial_v, final_v;
	double v, phi;
	double v_increment;

	initial_v	  = -GlobalState::robot_config.max_vel;
	final_v		  = fmin(GlobalState::robot_config.max_vel, 2.0); //limitando a velocidade maxima da ré para 2 m/s
	v_increment	  = 1.0;

	//fill velocity_search_vector
	for (v = initial_v; v <= final_v; v += v_increment)
	{
		velocity_search_vector.push_back(-v);
	}

	minimal_velocity_search_vector.push_back(GlobalState::robot_config.max_vel);
	minimal_velocity_search_vector.push_back(0.0);
	minimal_velocity_search_vector.push_back(-fmin(GlobalState::robot_config.max_vel, 0.5));

	phi_mask_vector.push_back(0.0);
	for (phi = get_phi_increment(0); phi <= (GlobalState::robot_config.max_phi * 2); phi += get_phi_increment(phi))
		phi_mask_vector.push_back(phi);

	RRT_Node::n = 40;
	RRT_Node::k = 4;
}


void 
predict_next_step(Pose &new_pose, const Command &command, const double &time)
{
	new_pose.x	   += command.v * time * cos(new_pose.theta);
	new_pose.y	   += command.v * time * sin(new_pose.theta);
	new_pose.theta += command.v * time * tan(command.phi) / GlobalState::robot_config.distance_between_front_and_rear_axles;
}


Pose 
Ackerman::predict_next_pose(const Pose &pose, const Command &command, double interval_time)
{
	Pose   new_pose;
	double time, rest;
	int	   n;

	new_pose = pose;
	time = 0.01;
	if (interval_time < time)
	{
		predict_next_step(new_pose, command, interval_time);
	}
	else
	{
		n	 = interval_time / time;
		rest = interval_time - (n * time);

		// euler method
		for (int i = 0; i < n; i++)
			predict_next_step(new_pose, command, time);

		predict_next_step(new_pose, command, rest);
	}
	new_pose.theta = carmen_normalize_theta(new_pose.theta);

	return new_pose;
}


Pose 
Ackerman::predict_next_pose(const Pose &pose, const Command &command, double traveled_dist, double *time_spend)
{
	Pose   new_pose, last_pose;
	double time;
	double distance;

	time = 0.01;
	distance = 0.0;
	*time_spend = 0.0;

	last_pose = new_pose = pose;

	do {
		predict_next_step(new_pose, command, time);
		*time_spend += time;
		distance += last_pose.distance(new_pose);
		last_pose = new_pose;
	} while ((distance < traveled_dist) && (*time_spend < 5.0));

	new_pose.theta = carmen_normalize_theta(new_pose.theta);

	return new_pose;
}


double 
Ackerman::get_turning_radius(double phi, double distance_between_front_and_rear_axles)
{
	return distance_between_front_and_rear_axles / tan(fabs(phi));
}


double 
Ackerman::get_acceleration_magnitude(const double &v, const double &desired_v)
{
	double acceleration = 0.0;

	if (fabs(desired_v) > fabs(v))
	{
		acceleration = GlobalState::robot_config.desired_acceleration;
	}
	else if (fabs(desired_v) < fabs(v))
	{
		acceleration = (desired_v >= 0) ? GlobalState::robot_config.desired_decelaration_forward : GlobalState::robot_config.desired_decelaration_reverse;
	}

	return acceleration;
}


double 
get_max_v_change(const double &current_v, const double &desired_v, const double &time)
{
	double time_spend;
	double v_change;
	double acceleration;
	int signal_current_v, signal_desired_v;

	signal_current_v = Util::signal(current_v);
	signal_desired_v = Util::signal(desired_v);

	if (signal_current_v == signal_desired_v ||
	    fabs(current_v) < 0.001 || fabs(desired_v) < 0.001)
	{
		acceleration = Ackerman::get_acceleration_magnitude(current_v, desired_v);

		v_change = fabs(acceleration * time);
	}
	else
	{
		acceleration = Ackerman::get_acceleration_magnitude(current_v, 0);
		//desacelerar até 0
		v_change = fmin(fabs(current_v), acceleration * time);

		time_spend = fabs(v_change / acceleration);

		if ((time - time_spend) > 0)
		{
			acceleration = Ackerman::get_acceleration_magnitude(0, desired_v);
			v_change += fabs(acceleration * (time - time_spend));
		}
	}

	return v_change;
}


double
predict_next_step(Robot_State &new_robot_state, const Command &command, const double &time,
		double &achieved_curvature, const double &desired_curvature, double &max_curvature_change)
{
	Robot_State initial_robot_state = new_robot_state;

	double delta_curvature = fabs(achieved_curvature - desired_curvature);
	double command_curvature_signal = (achieved_curvature < desired_curvature) ? 1.0 : -1.0;

	// As linhas de codigo abaixo assumem que a curvatura (phi) vai mudar dentro de time de delta_curvature ou max_curvature_change, o que for menor.
	// Ou seja, que o volante vai acelerar ate a velocidade maxima, e desacelerar ate parar na nova curvatura.
	// Tem que colocar um Euler aqui e fazer o mesmo no caso do rrt_path_follower.
	achieved_curvature = achieved_curvature + command_curvature_signal * fmin(delta_curvature, max_curvature_change);
	new_robot_state.v_and_phi.phi = carmen_get_phi_from_curvature(
			achieved_curvature, initial_robot_state.v_and_phi.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	// Tem que checar se as equacoes que governam esta mudancca de v estao corretas (precisa de um Euler?) e fazer o mesmo no caso do rrt_path_follower.
	double max_v_change = get_max_v_change(initial_robot_state.v_and_phi.v, command.v, time);
	double delta_v = fabs(initial_robot_state.v_and_phi.v - command.v);
	double command_v_signal = (initial_robot_state.v_and_phi.v < command.v) ? 1.0 : -1.0;
	new_robot_state.v_and_phi.v = initial_robot_state.v_and_phi.v + command_v_signal * fmin(delta_v, max_v_change);

	double move_x = initial_robot_state.v_and_phi.v * time * cos(initial_robot_state.pose.theta);
	double move_y = initial_robot_state.v_and_phi.v * time * sin(initial_robot_state.pose.theta);

	new_robot_state.pose.x	   += move_x;
	new_robot_state.pose.y	   += move_y;
	new_robot_state.pose.theta += initial_robot_state.v_and_phi.v * time * tan(initial_robot_state.v_and_phi.phi) / GlobalState::robot_config.distance_between_front_and_rear_axles;

	return sqrt(move_x * move_x + move_y * move_y);
}


Robot_State
Ackerman::predict_next_pose_dist(const Robot_State &robot_state, const Command &command, double distance_interval, double *time_spend)
{
	Robot_State new_robot_state;
	double new_curvature, curvature, max_curvature_change, time;
	double distance;

	distance = 0.0;
	*time_spend = 0.0;

	time = 0.01;
	new_robot_state = robot_state;

	curvature = carmen_get_curvature_from_phi(
			command.phi, command.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	new_curvature = carmen_get_curvature_from_phi(
			new_robot_state.v_and_phi.phi, new_robot_state.v_and_phi.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	max_curvature_change = GlobalState::robot_config.desired_steering_command_rate * time;

	//euler method
	while (*time_spend < 5.0)
	{
		distance += predict_next_step(new_robot_state, command, time, new_curvature, curvature, max_curvature_change);

		if (distance > distance_interval || distance == 0.0)
			break;

		*time_spend += time;
	}

	return new_robot_state;
}


void
Ackerman::compute_intermediate_times(double &t_fim_subida, double &t_fim_plato, double &t_fim_descida, double &max_phi_acceleration,
		double initial_phi, double requested_phi, const double full_time_interval, double &max_phi_velocity)
{
	t_fim_subida = max_phi_velocity / max_phi_acceleration; // V = at; t = V / a

	if (t_fim_subida >= (full_time_interval / 2.0)) // (D) e (E) Nao vai dar tempo para alcancar a max_phi_velocity
	{
		max_phi_velocity = 0.0;
		t_fim_subida = full_time_interval / 2.0;
		t_fim_plato = t_fim_subida;
		t_fim_descida = full_time_interval;
		// S = 2.0 * at²/2 + vt // Tempos com aceleracao constante (subida e descida) mais tempo com velocidade constante no plato
		double delta_phi_na_subida_mais_descida = max_phi_acceleration * t_fim_subida * t_fim_subida; // S = 2.0 * at²/2
		if (delta_phi_na_subida_mais_descida > fabs(requested_phi - initial_phi)) // (E) Se usar a aceleracao maxima vai ultrapassar o phi desejado
			max_phi_acceleration = fabs(requested_phi - initial_phi) / (t_fim_subida * t_fim_subida); // S = 2.0 * at²/2; a = S / (2.0 * t²/2)
	}
	else
	{
		// S = 2.0 * at²/2 + vt // Tempos com aceleracao constante (subida e descida) mais tempo com velocidade constante no plato
		double delta_phi_na_subida_mais_descida = max_phi_acceleration * t_fim_subida * t_fim_subida; // S = 2.0 * at²/2
		double tempo_maximo_de_plato = full_time_interval - 2.0 * t_fim_subida;
		double delta_phi_alcancavel = delta_phi_na_subida_mais_descida + tempo_maximo_de_plato * max_phi_velocity; // S = 2.0 * at²/2 + vt

		if (delta_phi_alcancavel <= fabs(requested_phi - initial_phi)) // Nao da para alcancar o requested_phi
		{
			t_fim_plato = full_time_interval - t_fim_subida;
			t_fim_descida = full_time_interval;
		}
		else if ((fabs(requested_phi - initial_phi) - delta_phi_na_subida_mais_descida) < 0.0) // Vai alcancar requested_phi antes do full_time_interval sem precisar atingir velocidade maxima
		{
			// S = at²/2; at² = 2 * S; t² = (2 * S) / a; t = sqrt((2 * S) / a)
			t_fim_subida = sqrt(fabs(requested_phi - initial_phi) / max_phi_acceleration);
			t_fim_plato = t_fim_subida;
			t_fim_descida = 2.0 * t_fim_subida;
		}
		else // Vai alcancar o requested_phi e vai alcancar a velocidade maxima
		{
			double delta_phi_no_plato = fabs(requested_phi - initial_phi) - delta_phi_na_subida_mais_descida;
			// S = vt; t = S / v;
			double tempo_de_plato = delta_phi_no_plato / max_phi_velocity;
			t_fim_plato = t_fim_subida + tempo_de_plato;
			t_fim_descida = t_fim_plato + t_fim_subida;
		}
	}
}


double
get_max_phi_change(double initial_phi, double requested_phi, double &phi_velocity,
		const double t, const double delta_t,
		double t_fim_descida, double t_fim_plato, double t_fim_subida,
		double max_phi_velocity, double max_phi_acceleration)
{
/*	if (fabs(requested_phi - initial_phi) < 0.000001)
	{
		phi_velocity = 0.0;

		return (requested_phi - initial_phi);
	}
*/
	if ((requested_phi - initial_phi) < 0)
	{
		max_phi_velocity = -max_phi_velocity;
		max_phi_acceleration = -max_phi_acceleration;
	}

	double max_phi_change = 0.0;
	if ((t + delta_t) <= t_fim_subida) // (a) t esta na subida e t + delta_t esta dentro da subida
	{
		max_phi_change = phi_velocity * delta_t + 0.5 * max_phi_acceleration * delta_t * delta_t; // S = V0t + at²/2
		phi_velocity = phi_velocity + max_phi_acceleration * delta_t; // V = V0 + at

		return (max_phi_change);
	}
	else if ((t < t_fim_subida) && ((t + delta_t) <= t_fim_plato)) // (b) t esta na subida, mas t + delta_t esta no plato
	{
		double delta_t_na_subida = t_fim_subida - t;
		max_phi_change = phi_velocity * delta_t_na_subida + 0.5 * max_phi_acceleration * delta_t_na_subida * delta_t_na_subida; // S = V0t + at²/2

		double delta_t_no_plato = t + delta_t - t_fim_subida;
		max_phi_change += max_phi_velocity * delta_t_no_plato; // S = S0 + Vt

		phi_velocity = max_phi_velocity; // plato de velocidade

		return (max_phi_change);
	}
	else if ((t < t_fim_subida) && ((t + delta_t) > t_fim_plato) && ((t + delta_t) <= t_fim_descida)) // (c) t esta na subida e t + delta_t esta depois do plato
	{
		double delta_t_na_subida = t_fim_subida - t;
		max_phi_change = phi_velocity * delta_t_na_subida + 0.5 * max_phi_acceleration * delta_t_na_subida * delta_t_na_subida; // S = V0t + at²/2
		phi_velocity = phi_velocity + max_phi_acceleration * delta_t_na_subida;

		double delta_t_no_plato = t_fim_plato - t_fim_subida;
		if (delta_t_no_plato != 0.0)
		{
			max_phi_change += max_phi_velocity * delta_t_no_plato; // S = S0 + Vt
			phi_velocity = max_phi_velocity;
		}
		double delta_t_na_descida = t + delta_t - t_fim_plato;
		max_phi_change += phi_velocity * delta_t_na_descida - 0.5 * max_phi_acceleration * delta_t_na_descida * delta_t_na_descida; // S = V0t + at²/2

		phi_velocity = phi_velocity - max_phi_acceleration * delta_t_na_descida; // V = V0 + at

		return (max_phi_change);
	}
	else if ((t >= t_fim_subida) && (t <= t_fim_plato) && ((t + delta_t) <= t_fim_plato)) // (d) t e t + delta_t estao no plato
	{
		max_phi_change = max_phi_velocity * delta_t; // S = Vt
		phi_velocity = max_phi_velocity; // plato de velocidade

		return (max_phi_change);
	}
	else if ((t >= t_fim_subida) && (t <= t_fim_plato) && ((t + delta_t) <= t_fim_descida)) // (e) t esta no plato, mas t + delta_t esta depois do plato
	{
		double delta_t_no_plato = t_fim_plato - t;
		max_phi_change = max_phi_velocity * delta_t_no_plato; // S = S0 + Vt

		double delta_t_na_descida = t + delta_t - t_fim_plato;
		max_phi_change += max_phi_velocity * delta_t_na_descida - 0.5 * max_phi_acceleration * delta_t_na_descida * delta_t_na_descida; // S = V0t + at²/2

		phi_velocity = max_phi_velocity - max_phi_acceleration * delta_t_na_descida; // V = V0 + at

		return (max_phi_change);
	}
	else if ((t >= t_fim_plato) && ((t + delta_t) <= t_fim_descida)) // (f) t e t + delta_t estao na descida
	{
		max_phi_change = phi_velocity * delta_t - 0.5 * max_phi_acceleration * delta_t * delta_t; // S = V0t + at²/2
		phi_velocity = phi_velocity - max_phi_acceleration * delta_t; // V = V0 + at

		return (max_phi_change);
	}
	else if ((t >= t_fim_plato) && (t < t_fim_descida) && ((t + delta_t) > t_fim_descida)) // (g) t esta na descida, mas t + delta_t esta depois do fim da descida
	{
		double delta_t_na_descida = t_fim_descida - t;
		max_phi_change = phi_velocity * delta_t_na_descida - 0.5 * max_phi_acceleration * delta_t_na_descida * delta_t_na_descida; // S = V0t + at²/2

		phi_velocity = 0.0;

		return (max_phi_change);
	}
	else if (t >= t_fim_descida) // t e t + delta_t estao depois do fim da descida
	{
		max_phi_change = 0.0;

		phi_velocity = 0.0;

		return (max_phi_change);
	}
	else if ((t < t_fim_subida) && ((t + delta_t) > t_fim_descida)) // (i) t esta na subida e t + delta_t esta depois do fim da descida
	{
		double delta_t_na_subida = t_fim_subida - t;
		max_phi_change = phi_velocity * delta_t_na_subida + 0.5 * max_phi_acceleration * delta_t_na_subida * delta_t_na_subida; // S = V0t + at²/2
		phi_velocity = phi_velocity + max_phi_acceleration * delta_t_na_subida;

		double delta_t_no_plato = t_fim_plato - t_fim_subida;
		if (delta_t_no_plato != 0.0)
		{
			max_phi_change += max_phi_velocity * delta_t_no_plato; // S = S0 + Vt
			phi_velocity = max_phi_velocity;
		}
		double delta_t_na_descida = t_fim_descida - t_fim_plato;
		max_phi_change += phi_velocity * delta_t_na_descida - 0.5 * max_phi_acceleration * delta_t_na_descida * delta_t_na_descida; // S = V0t + at²/2

		// phi_velocity deveria ser igual a zero (ou muito proximo) depois desta linha abaixo
		phi_velocity = phi_velocity - max_phi_acceleration * delta_t_na_descida; // V = V0 + at

		phi_velocity = 0.0;

		return (max_phi_change);
	}
	else if ((t >= t_fim_subida) && (t <= t_fim_plato) && ((t + delta_t) > t_fim_descida)) // (j) t esta no plato, mas t + delta_t esta depois do fim da descida
	{
		double delta_t_no_plato = t_fim_plato - t;
		max_phi_change = max_phi_velocity * delta_t_no_plato; // S = S0 + Vt

		double delta_t_na_descida = t_fim_descida - t_fim_plato;
		max_phi_change += max_phi_velocity * delta_t_na_descida - 0.5 * max_phi_acceleration * delta_t_na_descida * delta_t_na_descida; // S = V0t + at²/2

		phi_velocity = max_phi_velocity - max_phi_acceleration * delta_t_na_descida; // V = V0 + at

		return (max_phi_change);
	}
	else
	{
		printf("Nao deveria acontecer 1 em get_max_phi_change()\n");
		exit(1);
	}

	// Precisamos saber se dentro de time o phi chega ao requested_phi respeitando os limites de velocidade e aceleracao de phi
	// Existem 3 situacoes: manter a phi_velocity == max_phi_velocity, aumenta-la ou diminui-la. Tem que descobrir qual eh a situacao e agir de acordo.
	// Se a situacao (phi_velocity == max_phi_velocity) deve ser mantida, avancar phi segundo esta velocidade na direcao de requested_phi
	// Se phi_velocity deve ser aumentada na direcao de max_phi_velocity, aumenta-la segundo max_phi_acceleration e
	// 	ajustar phi na direcao de requested_phi considerando o aumento linear de phi_velocity segundo max_phi_acceleration.
	// Se phi_velocity deve ser diminuida na direcao de -max_phi_velocity, diminui-la segundo max_phi_acceleration e
	// 	ajustar phi na direcao de requested_phi considerando a diminuicao linear de phi_velocity segundo max_phi_acceleration.
	return 0.0;
}


#ifdef OLD_STEERING_CONTROL
double
Ackerman::predict_next_pose_during_main_rrt_planning_step(Robot_State &new_robot_state, const Command &requested_command, double delta_t,
		double &achieved_curvature, const double &desired_curvature, double &max_curvature_change)
#else
double
Ackerman::predict_next_pose_during_main_rrt_planning_step(Robot_State &new_robot_state, const Command &requested_command, double &phi_velocity,
		const double t, const double delta_t,
		double t_fim_descida, double t_fim_plato, double t_fim_subida,
		double max_phi_velocity, double max_phi_acceleration)
#endif
{
	Robot_State initial_robot_state = new_robot_state;

#ifdef OLD_STEERING_CONTROL
	double delta_curvature = fabs(achieved_curvature - desired_curvature);
	double command_curvature_signal = (achieved_curvature < desired_curvature) ? 1.0 : -1.0;

	achieved_curvature = achieved_curvature + command_curvature_signal * fmin(delta_curvature, max_curvature_change);
	new_robot_state.v_and_phi.phi = carmen_get_phi_from_curvature(
			achieved_curvature, initial_robot_state.v_and_phi.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);
#else
	// Tem que checar se as equacoes que governam esta mudancca de phi estao corretas (precisa de um Euler?) e fazer o mesmo no caso do rrt_path_follower.
	double max_phi_change = get_max_phi_change(initial_robot_state.v_and_phi.phi, requested_command.phi, phi_velocity,
			t, delta_t,
			t_fim_descida, t_fim_plato, t_fim_subida,
			max_phi_velocity, max_phi_acceleration);
	new_robot_state.v_and_phi.phi = initial_robot_state.v_and_phi.phi + max_phi_change;
#endif

	// Tem que checar se as equacoes que governam esta mudancca de v estao corretas (precisa de um Euler?) e fazer o mesmo no caso do rrt_path_follower.
	double max_v_change = get_max_v_change(initial_robot_state.v_and_phi.v, requested_command.v, delta_t);
	double delta_v = fabs(initial_robot_state.v_and_phi.v - requested_command.v);
	double command_v_signal = (initial_robot_state.v_and_phi.v < requested_command.v) ? 1.0 : -1.0;
	new_robot_state.v_and_phi.v = initial_robot_state.v_and_phi.v + command_v_signal * fmin(delta_v, max_v_change);

	double move_x = initial_robot_state.v_and_phi.v * delta_t * cos(initial_robot_state.pose.theta);
	double move_y = initial_robot_state.v_and_phi.v * delta_t * sin(initial_robot_state.pose.theta);

	new_robot_state.pose.x	   += move_x;
	new_robot_state.pose.y	   += move_y;
	new_robot_state.pose.theta += initial_robot_state.v_and_phi.v * delta_t * tan(initial_robot_state.v_and_phi.phi) / GlobalState::robot_config.distance_between_front_and_rear_axles;

	carmen_robot_and_trailer_traj_point_t robot_and_trailer_traj_point = {
			new_robot_state.pose.x, new_robot_state.pose.y, new_robot_state.pose.theta, new_robot_state.pose.beta,
			new_robot_state.v_and_phi.v, new_robot_state.v_and_phi.phi
	};
	carmen_robot_ackerman_config_t robot_config;
	robot_config.distance_between_front_and_rear_axles = GlobalState::robot_config.distance_between_front_and_rear_axles;
	new_robot_state.pose.beta  = compute_semi_trailer_beta(robot_and_trailer_traj_point, delta_t,
			robot_config, GlobalState::semi_trailer_config);

	return sqrt(move_x * move_x + move_y * move_y);
}


Robot_State
Ackerman::predict_next_pose_during_main_rrt_planning(const Robot_State &robot_state, const Command &requested_command,
		double full_time_interval, double *distance_traveled, double delta_t)
{
	if (distance_traveled)
		*distance_traveled = 0.0;

	int n = floor(full_time_interval / delta_t);
	double remaining_time = full_time_interval - ((double) n * delta_t);
	Robot_State achieved_robot_state = robot_state; // achieved_robot_state eh computado iterativamente abaixo a partir do estado atual do robo

#ifdef OLD_STEERING_CONTROL
	double curvature = carmen_get_curvature_from_phi(
			requested_command.phi, requested_command.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	double new_curvature = carmen_get_curvature_from_phi(
			achieved_robot_state.v_and_phi.phi, achieved_robot_state.v_and_phi.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	double max_curvature_change = GlobalState::robot_config.desired_steering_command_rate * delta_t;
#else
	double max_phi_velocity = GlobalState::max_phi_velocity;
	double max_phi_acceleration = GlobalState::max_phi_acceleration;

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
				new_curvature, curvature, max_curvature_change);
#else
		double t = (double) i * delta_t;
		double dist_walked = predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, phi_velocity,
				t, delta_t,
				t_fim_descida, t_fim_plato, t_fim_subida,
				max_phi_velocity, max_phi_acceleration);
#endif

		if (distance_traveled)
			*distance_traveled += dist_walked;
	}

	if (remaining_time > 0.0)
	{
#ifdef OLD_STEERING_CONTROL
		double dist_walked = predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, delta_t,
				new_curvature, curvature, max_curvature_change);
#else
		double t = (double) n * delta_t;
		double dist_walked = predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, phi_velocity,
				t, remaining_time,
				t_fim_descida, t_fim_plato, t_fim_subida,
				max_phi_velocity, max_phi_acceleration);
#endif

		if (distance_traveled)
			*distance_traveled += dist_walked;
	}

	achieved_robot_state.pose.theta = carmen_normalize_theta(achieved_robot_state.pose.theta);

	return achieved_robot_state;
}


bool
Ackerman::remove_front_node_of_the_path(Pose &robot_pose, list<RRT_Path_Edge> &path,
		double &path_distance, double &theta_diff,
		Robot_State *path_pose, double *time_to_reach_path_pose, double *distance_traveled_rtr,
		double *total_distance_rtr)
{
	//todo not good enough

	if (path.empty())
		return false;

	static double porcentage_threshold = 0.98;
	static double distance_threshold = 0.1;
	static double phi_threshold = carmen_degrees_to_radians(3.5);
	static double v_threshold = 0.05;

	double time_spend;
	double followed_porcentage, traveled_dist, total_dist;
	Robot_State pose_in_the_curve_p1_p2_closest_to_the_robot_pose;
	Command command_of_the_first_node_of_the_path;
	double delta_distance, phi_diff, v_diff;
	list<RRT_Path_Edge>::iterator current;

	command_of_the_first_node_of_the_path = path.front().command;

	path_distance = Ackerman::distance_from_robot_pose_to_curve_between_p1_and_p2(robot_pose, path.front().p1,
			path.front().p2, path.front().command, &traveled_dist, &total_dist,
			&pose_in_the_curve_p1_p2_closest_to_the_robot_pose, &time_spend);

	theta_diff = robot_pose.get_theta_diff(pose_in_the_curve_p1_p2_closest_to_the_robot_pose.pose);
	phi_diff = fabs(carmen_normalize_theta(GlobalState::last_odometry.phi - command_of_the_first_node_of_the_path.phi));
	v_diff = fabs(GlobalState::last_odometry.v - command_of_the_first_node_of_the_path.v);

	if (total_dist == 0.0)
		followed_porcentage = 0.0;
	else
		followed_porcentage = (traveled_dist / total_dist);

	delta_distance = robot_pose.distance(path.front().p2.pose);

	//verify if it is necessary to change the root_node
	if (path.front().p1.pose == path.front().p2.pose)
	{
		if (phi_diff < phi_threshold)
		{
			path.pop_front();

			if (!path.empty())
			{
				if (path_pose)
					*path_pose = path.begin()->p1;
				if (time_to_reach_path_pose)
					*time_to_reach_path_pose = 0.0;
				if (distance_traveled_rtr)
					*distance_traveled_rtr = 0.0;
				if (total_distance_rtr)
					*total_distance_rtr = 0.0;
			}

			return true;
		}
	}
	else if (followed_porcentage > porcentage_threshold ||
			delta_distance < distance_threshold ||
//			delta_distance > (total_dist - traveled_dist + distance_threshold * 4) ||
			(command_of_the_first_node_of_the_path.v == 0 && v_diff < v_threshold))
	{
		path.pop_front();

		if (!path.empty())
		{
			if (path_pose)
				*path_pose = path.begin()->p1;
			if (time_to_reach_path_pose)
				*time_to_reach_path_pose = 0.0;
			if (distance_traveled_rtr)
				*distance_traveled_rtr = 0.0;
			if (total_distance_rtr)
				*total_distance_rtr = -1.0;
		}

		return true;
	}

	if (path_pose)
		*path_pose = pose_in_the_curve_p1_p2_closest_to_the_robot_pose;

	if (time_to_reach_path_pose)
		*time_to_reach_path_pose = time_spend;

	if (distance_traveled_rtr)
		*distance_traveled_rtr = traveled_dist;

	if (total_distance_rtr)
		*total_distance_rtr = total_dist;

	return false;
}


bool 
get_stop_command(Robot_State &robot_state, Command &command, double &time_spend)
{
	double distance_traveled;

	command.v = 0.0;
	command.phi = robot_state.v_and_phi.phi;

	Ackerman::get_stop_info(robot_state, robot_state.v_and_phi.phi, &distance_traveled, &time_spend);

	if (!Obstacle_Detection::is_obstacle_path2(robot_state, command, time_spend))
		return true;

	//pesquisar outros comandos!

	return false;
}


double 
Ackerman::distance_from_robot_pose_to_curve_between_p1_and_p2(Pose robot_pose, Robot_State p1, Robot_State p2,
		Command command, double *traveled_dist, double *total_dist, Robot_State *closest_path_pose, double *time_spend)
{
	double interval_time = 0.01;
	double total_time = 0.0;
	double dist_to_p2, new_dist_to_p2, dist_to_closest_pose, new_dist, dist;
	Robot_State rs, new_rs;

	rs = p1;
	dist_to_closest_pose = robot_pose.distance(rs.pose);
	if (closest_path_pose)
		*closest_path_pose = rs;
	dist_to_p2 = rs.distance(p2);

	*traveled_dist = 0;
	*total_dist = 0;
	if (time_spend)
		*time_spend = 0;

	while (total_time < 5.0)
	{
		new_rs = Ackerman::predict_next_pose_during_main_rrt_planning(rs, command, interval_time);
		dist = new_rs.distance(rs);

		*total_dist += dist;

		total_time += interval_time;

		rs = new_rs;

		new_dist_to_p2 = rs.distance(p2);

		if (new_dist_to_p2 > dist_to_p2)
			break;

		dist_to_p2 = new_dist_to_p2;

		new_dist = robot_pose.distance(rs.pose);

		if (new_dist < dist_to_closest_pose)
		{
			dist_to_closest_pose = new_dist;
			*traveled_dist = *total_dist;

			if (time_spend)
				*time_spend = total_time;

			if (closest_path_pose)
				*closest_path_pose = rs;
		}

		if (dist == 0)
			break;
	}

	return dist_to_closest_pose;
}


bool
Ackerman::is_valid_phi(double current_v, double current_phi, double desired_v, double desired_phi, double time)
{
	double current_curvature, desired_curvature;
	double curvature_diff;

	if (fabs(desired_phi) > GlobalState::robot_config.max_phi)
		return false;

	current_curvature = carmen_get_curvature_from_phi(current_phi, current_v,
			GlobalState::robot_config.understeer_coeficient, GlobalState::robot_config.distance_between_front_and_rear_axles);

	desired_curvature = carmen_get_curvature_from_phi(desired_phi, desired_v,
			GlobalState::robot_config.understeer_coeficient, GlobalState::robot_config.distance_between_front_and_rear_axles);

	curvature_diff = fabs(current_curvature - desired_curvature);

	return curvature_diff <= (GlobalState::robot_config.desired_steering_command_rate * time);
}


void 
Ackerman::get_stop_info(Robot_State robot_state, double phi_command, double *traveled_distance, double *time_spend)
{
	Robot_State new_robot_state;
	Command command;
	double new_curvature, curvature, max_curvature_change, time;
	double dist_increment;

	*time_spend = 0.0;
	*traveled_distance = 0.0;

	command.v = 0;
	command.phi = phi_command;

	time = 0.1;
	new_robot_state = robot_state;

	curvature = carmen_get_curvature_from_phi(
			command.phi, command.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	new_curvature = carmen_get_curvature_from_phi(
			new_robot_state.v_and_phi.phi, new_robot_state.v_and_phi.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	max_curvature_change = GlobalState::robot_config.desired_steering_command_rate * time;

	//euler method
	while (*time_spend < 5.0)
	{
		predict_next_step(new_robot_state, command, time, new_curvature, curvature, max_curvature_change);

		dist_increment = robot_state.distance(new_robot_state);

		*traveled_distance += dist_increment;
		*time_spend += time;

		if (new_robot_state.v_and_phi.v == 0 || dist_increment == 0)
			return;

		robot_state = new_robot_state;
	}
}


double
increase_velocity_under_constant_acceleration(double *velocity_reached, double *traveled_distance, double v0, double desired_vel,
		double distance, double acceleration_magnitude, double total_time)
{	// Esta fucao so funciona com v0 e desired_vel >= 0
	*traveled_distance = distance;

	// V² = Vo² + 2.a.(S-S0) -> http://physics.bu.edu/~redner/211-sp06/class01/equations.html
	*velocity_reached = sqrt(v0 * v0 + 2 * acceleration_magnitude * distance);
	if (*velocity_reached <= desired_vel) // Nao vai alcancar a velocidade desejada, mas apenas aumentar a velocidade no espaco percorrido
	{
		// v = a * t;  t = v / a
		total_time = (*velocity_reached - v0) / acceleration_magnitude;
	}
	else // Vai alcanccar e a velocidade desejada dentro do intervalo percorrido e ficar nela um tempo
	{
		*velocity_reached = desired_vel;
		// V = Vo + a.t; t = (V - Vo) / a -> tempo ate alcancar a velocidade desejada
		double intermediate_time = (desired_vel - v0) / acceleration_magnitude;
		// s = vo * t + (a * t²) / 2.0
		double traveled_dist = v0 * intermediate_time + 0.5 * acceleration_magnitude * intermediate_time * intermediate_time; // Distancia necessaria para alcancar a velocidade desejada
				// s = v * t; t = s / v
		total_time = intermediate_time + (distance - traveled_dist) / desired_vel; // Tempo necessario para percorrer o restante da distancia (3.5) apos alcancar a velocidade desejada
	}

	return total_time;
}


double
decrease_velocity_under_constant_acceleration(double *velocity_reached, double *traveled_distance, double v0, double desired_vel,
		double distance, double acceleration_magnitude, double total_time)
{	// Esta fucao so funciona com v0 e desired_vel >= 0
	// V² = Vo² + 2.a.(S-S0) -> http://physics.bu.edu/~redner/211-sp06/class01/equations.html
	*velocity_reached = sqrt(desired_vel * desired_vel + 2 * acceleration_magnitude * distance); // ATENCAO: Trocar v0 por desired_vel aqui eh o mesmo que andar para tras no tempo, mas neste caso nao tem problema porque o t foi retirado da equacao
	if (*velocity_reached <= v0) // Nao vai alcancar a velocidade desejada, mas apenas aumentar a velocidade no espaco percorrido
	{
		*traveled_distance = distance;

		*velocity_reached = sqrt(v0 * v0 - 2 * acceleration_magnitude * distance);
		// v = a * t;  t = v / a
		total_time = (v0 - *velocity_reached) / acceleration_magnitude;
	}
	else // Vai alcanccar e a velocidade desejada dentro do intervalo percorrido e ficar nela um tempo
	{
		*velocity_reached = desired_vel;
		// V = Vo + a.t; t = (V - Vo) / a -> tempo ate alcancar a velocidade desejada
		total_time = (v0 - desired_vel) / acceleration_magnitude;
		// s = vo * t + (a * t²) / 2.0
		*traveled_distance = v0 * total_time - 0.5 * acceleration_magnitude * total_time * total_time; // Distancia necessaria para alcancar a velocidade desejada
		if (desired_vel > 0.0)
		{
			// s = v * t; t = s / v
			total_time += (distance - *traveled_distance) / desired_vel; // Tempo necessario para percorrer o restante da distancia (3.5) apos alcancar a velocidade desejada
			*traveled_distance = distance;
		}
	}

	return total_time;
}


double
increase_negative_velocity_under_constant_acceleration(double *velocity_reached, double *traveled_distance, double v0, double desired_vel,
		double distance, double acceleration_magnitude, double total_time)
{	// Esta fucao so funciona com v0 e desired_vel < 0
	*traveled_distance = distance;

	// V² = Vo² + 2.a.(S-S0) -> http://physics.bu.edu/~redner/211-sp06/class01/equations.html
	*velocity_reached = -sqrt(v0 * v0 + 2 * acceleration_magnitude * distance);
	if (*velocity_reached > desired_vel) // Nao vai alcancar a velocidade desejada, mas apenas aumentar a velocidade no espaco percorrido
	{
		// v = a * t;  t = v / a
		total_time = (v0 - *velocity_reached) / acceleration_magnitude;
	}
	else // Vai alcanccar e a velocidade desejada dentro do intervalo percorrido e ficar nela um tempo
	{
		*velocity_reached = desired_vel;
		// V = Vo + a.t; t = (V - Vo) / a -> tempo ate alcancar a velocidade desejada
		double intermediate_time = (v0 - desired_vel) / acceleration_magnitude;
		// s = vo * t + (a * t²) / 2.0
		double traveled_dist = -v0 * intermediate_time + 0.5 * acceleration_magnitude * intermediate_time * intermediate_time; // Distancia necessaria para alcancar a velocidade desejada
				// s = v * t; t = s / v
		total_time = intermediate_time + (distance - traveled_dist) / (-desired_vel); // Tempo necessario para percorrer o restante da distancia (3.5) apos alcancar a velocidade desejada
	}

	return total_time;
}


double
decrease_negative_velocity_under_constant_acceleration(double *velocity_reached, double *traveled_distance, double v0, double desired_vel,
		double distance, double acceleration_magnitude, double total_time)
{	// Esta funcao so funciona com v0 e desired_vel negativas ou desired_vel = 0 e v0 < 0
	// V² = Vo² + 2.a.(S-S0) -> http://physics.bu.edu/~redner/211-sp06/class01/equations.html
	double velocity_reached_2 = v0 * v0 - 2 * acceleration_magnitude * distance;
	if (velocity_reached_2 > (desired_vel * desired_vel)) // Nao vai alcancar a velocidade desejada, mas apenas aumentar (ir em direcao a zero) a velocidade no espaco percorrido
	{
		*traveled_distance = distance;

		*velocity_reached = -sqrt(velocity_reached_2);
		// v = a * t;  t = v / a
		total_time = (*velocity_reached - v0) / acceleration_magnitude;
	}
	else // Vai alcanccar e a velocidade desejada dentro do intervalo percorrido e ficar nela um tempo
	{
		*velocity_reached = desired_vel;
		// V = Vo + a.t; t = (V - Vo) / a -> tempo ate alcancar a velocidade desejada
		total_time = (desired_vel - v0) / acceleration_magnitude;
		// S = Vo * t + (a * t²) / 2.0
		*traveled_distance = v0 * total_time + 0.5 * acceleration_magnitude * total_time * total_time; // Distancia necessaria para alcancar a velocidade desejada
		if (desired_vel != 0.0)
		{
			// s = v * t; t = s / v
			total_time += (distance - *traveled_distance) / (-desired_vel); // Tempo necessario para percorrer o restante da distancia (3.5) apos alcancar a velocidade desejada
			*traveled_distance = distance;
		}
	}

	return total_time;
}


double
mantain_velocity(double *velocity_reached, double *traveled_distance, double desired_vel, double distance, double total_time)
{
	if (desired_vel != 0.0)
	{
		*velocity_reached = desired_vel;
		*traveled_distance = distance;
		total_time = distance / fabs(desired_vel);
		return (total_time);
	}
	else
	{
		*velocity_reached = 0.0;
		*traveled_distance = 0.0;
		if (distance == 0)
			return (0.0);
		else
			return (DBL_MAX);
	}

	return total_time;
}


double
get_travel_time_step(double v0, double desired_vel, double distance, double *velocity_reached, double *traveled_distance)
{	// traveled_distance eh contada positivamente quando o carro se desloca na direcao de desired_vel, caso contrario, ela eh negativa
	double total_time = 0.0;
	double acceleration_magnitude = Ackerman::get_acceleration_magnitude(v0, desired_vel);
	if (acceleration_magnitude == 0.0)
	{
		total_time = mantain_velocity(velocity_reached, traveled_distance, desired_vel, distance, total_time);
		return (total_time);
	}

	if (desired_vel >= 0.0)
	{
		if (v0 >= 0.0)
		{
			if (desired_vel >= v0)
				total_time = increase_velocity_under_constant_acceleration(velocity_reached, traveled_distance, v0, desired_vel,
						distance, acceleration_magnitude, total_time);
			else // desired_vel < v0
				total_time = decrease_velocity_under_constant_acceleration(velocity_reached, traveled_distance, v0, desired_vel,
						distance, acceleration_magnitude, total_time);
		}
		else // v0 < 0.0
		{
			total_time = decrease_negative_velocity_under_constant_acceleration(velocity_reached, traveled_distance, v0, 0.0,
					distance, acceleration_magnitude, total_time);
			if (*velocity_reached == 0.0)
			{
				double distance_up_to_zero_v = *traveled_distance;
				total_time += GlobalState::time_to_change_gears;
				total_time += increase_velocity_under_constant_acceleration(velocity_reached, traveled_distance, 0.0, desired_vel,
						distance - distance_up_to_zero_v, acceleration_magnitude, total_time);
				*traveled_distance += distance_up_to_zero_v;
			}
		}
	}
	else // desired_vel < 0.0
	{
		if (v0 < 0.0)
		{
			if (desired_vel < v0)
				total_time = increase_negative_velocity_under_constant_acceleration(velocity_reached, traveled_distance, v0, desired_vel,
						distance, acceleration_magnitude, total_time);
			else // desired_vel >= v0
				total_time = decrease_negative_velocity_under_constant_acceleration(velocity_reached, traveled_distance, v0, desired_vel,
						distance, acceleration_magnitude, total_time);
		}
		else // v0 >= 0.0
		{
			total_time = decrease_velocity_under_constant_acceleration(velocity_reached, traveled_distance, v0, 0.0,
					distance, acceleration_magnitude, total_time);
			if (*velocity_reached == 0.0)
			{
				double distance_up_to_zero_v = *traveled_distance;
				total_time += GlobalState::time_to_change_gears;
				total_time += increase_negative_velocity_under_constant_acceleration(velocity_reached, traveled_distance, 0.0, desired_vel,
						distance - distance_up_to_zero_v, acceleration_magnitude, total_time);
				*traveled_distance += distance_up_to_zero_v;
			}
		}
	}

	return total_time;
}


double
get_travel_time_step_old(double v0, double desired_vel, double distance, double *velocity_reached, double *traveled_distance)
{
	double total_time = 0.0;

	double acceleration = Ackerman::get_acceleration_magnitude(v0, desired_vel);
	*traveled_distance = distance;

	if (fabs(desired_vel) < fabs(v0))
		acceleration = -acceleration;

	// V² = Vo² + 2.a.(S-S0)
	double velocity_reached_2 = pow(v0, 2) + 2 * acceleration * distance;

	if (velocity_reached_2 >= 0.0)
	{
		*velocity_reached = sqrt(velocity_reached_2) * Util::signal(desired_vel);

		if ((v0 == desired_vel) && (desired_vel != 0.0)) // Velocidade desejada eh igual a atual
		{
			*velocity_reached = desired_vel;

			// s = v * t; t = s / v
			total_time = distance / desired_vel;
		}
		else if (((acceleration > 0.0) && (fabs(*velocity_reached) <= fabs(desired_vel))) || // Nao vai alcancar a velocidade desejada, mas apenas aumentar a velocidade no espaco percorrido
				 ((acceleration < 0.0) && (fabs(*velocity_reached) >= fabs(desired_vel))))
		{
			// v = a * t;  t = v / a
			total_time = fabs((*velocity_reached - v0) / acceleration); // Alberto: Esta conta do tempo me parece errada...
		}
		else // Vai alcanccar a velocidade desejada dentro do intervalo percorrido e ficar nela um tempo
		{
			*velocity_reached = desired_vel;

			// V = Vo + a.t; t = (V - Vo) / a
			total_time = fabs((desired_vel - v0) / acceleration); // Tempo para chegar aa velocidade desejada

			// s = so + vo * t + (a * t^2) / 2.0
			double traveled_dist = fabs(fabs(v0) * total_time + 0.5 * acceleration * pow(total_time, 2)); // Distancia necessaria para alcancar a velocidade desejada

			// s = v * t; t = s / v
			total_time += fabs((distance - traveled_dist) / desired_vel); // Tempo necessario para percorrer o restante da distancia (3.5) apos alcancar a velocidade desejada
		}
	}
	else
	{
		//carro parou antes da distancia
		*velocity_reached = desired_vel;

		total_time = fabs((desired_vel - v0) / acceleration);

		double traveled_dist = fabs(fabs(v0) * total_time + (acceleration * pow(total_time, 2)) / 2.0);

		*traveled_distance = traveled_dist;
	}

	return total_time;
}


/*
double
Ackerman::get_travel_time_old(Robot_State robot_state, double desired_vel, double distance, double *velocity_reached)
{
	double traveled_distance;
	double time;

	if ((fabs(robot_state.v_and_phi.v) < 0.001) && (fabs(desired_vel) < 0.001))
	{
		*velocity_reached = 0.0;
		return 0.0;
	}

	if (Util::signal(robot_state.v_and_phi.v) == Util::signal(desired_vel) ||
		robot_state.v_and_phi.v == 0.0 || desired_vel == 0.0)
	{
		time = get_travel_time_step(robot_state.v_and_phi.v, desired_vel, distance, velocity_reached, &traveled_distance);
	}
	else
	{
		time = get_travel_time_step(robot_state.v_and_phi.v, 0.0, 		  distance, velocity_reached, &traveled_distance);

		if (traveled_distance < distance)
			time += get_travel_time_step(0, desired_vel, distance - traveled_distance, velocity_reached, &traveled_distance) + GlobalState::time_to_change_gears;
	}

	return time;
}
*/


double
Ackerman::get_travel_time(Robot_State robot_state, double desired_vel, double distance, double *velocity_reached)
{
	double traveled_distance;

	double time = get_travel_time_step(robot_state.v_and_phi.v, desired_vel, distance, velocity_reached, &traveled_distance);

	return time;
}


Command 
rs_move_to_command(const rs_move &move)
{
	Command c;

	c.v = move.move == RS_FWD ? 1.0 : -1.0;
	c.phi = 0;

	if (move.turn == RS_TURN_LEFT)
		c.phi = GlobalState::robot_config.max_phi;
	else if (move.turn == RS_TURN_RIGHT)
		c.phi = -GlobalState::robot_config.max_phi;

	return c;
}


void 
Ackerman::find_command_rs(Pose &p1, Pose &p2, vector<Command> &commands, vector<double> &commands_time)
{
	static double turning_radius = get_turning_radius(
			GlobalState::robot_config.max_phi,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	Pose current;
	Command command;
	RS_Path_Description path_description;
	int path_number;
	double tr, ur, vr, command_time, p1_p2_distance;

	reed_shepp(p1.x, p1.y, p1.theta, p2.x, p2.y, p2.theta, &path_number, &tr, &ur, &vr);

	if ((path_number - 1) < 0)
		return;

	rs_build_path_description(&path_description, path_number, tr, ur, vr);

	current = p1;

	//get commands
	for (int i = 0; i < path_description.size; i++)
	{
		command = rs_move_to_command(path_description.moves[i]);

		p1_p2_distance = command.phi == 0.0 ? path_description.dist[i] : path_description.dist[i] * turning_radius;

		command_time = fabs(p1_p2_distance / command.v);

		//		collision checking
		if (Obstacle_Detection::is_obstacle_path(current, command, command_time))
		{
			commands.clear();
			commands_time.clear();
			return;
		}

		commands.push_back(command);
		commands_time.push_back(command_time);

		current = Ackerman::predict_next_pose(current, command, command_time);
	}
}


void 
Ackerman::find_command_rs(Robot_State &robot_state, Pose &p2, vector<Command> &commands, vector<double> &commands_time)
{
	vector<Command> raw_commands;
	vector<double> raw_commands_time;
	Command c;
	Robot_State new_robot_state;
	Robot_State last_robot_state;
	double time_spend, current_curvature, desired_curvature, traveled_dist_to_stop, distance, time_to_stop;
	int signal_v;

	last_robot_state = robot_state;

	//	printf("current_v = %f\n", robot_state.command.v);

	//STOP the robot if necessary
	if (robot_state.v_and_phi.v == 0)
	{
		new_robot_state = robot_state;
	}
	else
	{
		if (get_stop_command(robot_state, c, time_spend))
		{
			new_robot_state = predict_next_pose_during_main_rrt_planning(last_robot_state, c, time_spend);

			commands.push_back(c);
			commands_time.push_back(time_spend);

			last_robot_state = new_robot_state;
		}
		else
			return;
	}


	find_command_rs(new_robot_state.pose, p2, raw_commands, raw_commands_time);

	for (unsigned int i = 0; i < raw_commands.size(); i++)
	{
		//s = v * t
		distance = fabs(raw_commands[i].v * raw_commands_time[i]);

		c = raw_commands[i];
		c.v = 0.0;

		current_curvature = carmen_get_curvature_from_phi(
				last_robot_state.v_and_phi.phi, last_robot_state.v_and_phi.v,
				GlobalState::robot_config.understeer_coeficient,
				GlobalState::robot_config.distance_between_front_and_rear_axles);

		desired_curvature = carmen_get_curvature_from_phi(
				c.phi, c.v,
				GlobalState::robot_config.understeer_coeficient,
				GlobalState::robot_config.distance_between_front_and_rear_axles);

		//obter tempo para virar a direcao
		time_spend = fabs((current_curvature - desired_curvature) / GlobalState::robot_config.desired_steering_command_rate);

		last_robot_state.v_and_phi.phi = c.phi; //mudar estado, para robo parado com a direcao certa

		//adicionando comando para virar a direcao
		commands.push_back(c);
		commands_time.push_back(time_spend);

		c = raw_commands[i];
		signal_v = Util::signal(c.v);
		int iter = 0;
		while (iter < 5000)
		{
			new_robot_state.v_and_phi = c;
			//obter a distancia que o robo percorre antes de parar com a velocidade c.v
			Ackerman::get_stop_info(new_robot_state, c.phi, &traveled_dist_to_stop, &time_to_stop);

			new_robot_state = predict_next_pose_dist(last_robot_state, c, distance - traveled_dist_to_stop, &time_spend);


			if (traveled_dist_to_stop >= distance)
			{
				c.v = -0.1 * signal_v + c.v;

				if (signal_v != Util::signal(c.v))
				{
					commands.clear();
					commands_time.clear();
					return;
				}

				continue;
			}

			//sair do loop apenas quando o robo conseguir alcancar a velocidade do comando
			//e parar no ponto certo
			if (c.v == new_robot_state.v_and_phi.v)
				break;

			c.v = new_robot_state.v_and_phi.v;
		}

		last_robot_state = new_robot_state;

		//adicionar comando para seguir até o ponto intermediario
		commands.push_back(c);
		commands_time.push_back(time_spend);

		//adicionar comando para parar no ponto final
		c.v = 0;
		c.phi = raw_commands[i].phi;
		new_robot_state = predict_next_pose_during_main_rrt_planning(last_robot_state, c, time_to_stop);
		last_robot_state = new_robot_state;
		commands.push_back(c);
		commands_time.push_back(time_to_stop);

		iter++;
	}
}


void 
get_velocity_vector_limits(const RRT_Node &near, const double &distance_interval, unsigned int &initial_v_index, unsigned int &final_v_index, const vector<double> &velocity_search_vector)
{
	double min_vel, max_vel;

	if (fabs(near.robot_state.v_and_phi.v) <= 0.005)
	{
		Ackerman::get_travel_time(near.robot_state, velocity_search_vector.back(), distance_interval, &min_vel);
		Ackerman::get_travel_time(near.robot_state, velocity_search_vector.front(), distance_interval, &max_vel);
	}
	else if(near.robot_state.v_and_phi.v > 0.0)
	{
		Ackerman::get_travel_time(near.robot_state, 0, distance_interval, &min_vel);
		Ackerman::get_travel_time(near.robot_state, velocity_search_vector.front(), distance_interval, &max_vel);
	}
	else
	{
		Ackerman::get_travel_time(near.robot_state, velocity_search_vector.back(), distance_interval, &min_vel);
		Ackerman::get_travel_time(near.robot_state, 0, distance_interval, &max_vel);
	}

	initial_v_index = 0;
	for (unsigned int i = 0; i < velocity_search_vector.size(); i++)
	{
		if (velocity_search_vector[i] < max_vel)
			break;
		initial_v_index = i;
	}

	final_v_index = velocity_search_vector.size() - 1;
	for (int i = (velocity_search_vector.size() - 1); i >= 0; i--)
	{
		if (velocity_search_vector[i] > min_vel)
			break;
		final_v_index = i;
	}
}


double
Ackerman::get_env_cost(const Robot_State robot_state)
{
	static double env_cost_weight = 1.0;

	if (GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_FOLLOW_ROUTE)
	{
		return (0.5 * Lane_Cost_Function::get_cost(robot_state) + 0.5 * Obstacle_Cost_Function2::get_cost(robot_state.pose, GlobalState::cost_map)) * env_cost_weight;
	}
	else
		return Obstacle_Cost_Function2::get_cost(robot_state.pose, GlobalState::cost_map) * env_cost_weight;
}


bool
Ackerman::search_command_improved_parking(RRT_Node &near, Pose &rand, RRT_Node *new_node, Command *best_command,
		double *best_command_time, Cost_Function **cost_functions, const double *weights, const int num_cost_function,
		double *min_cost, const vector<double> &velocity_search_vector, Tree &tree)
{
	Command_and_Cost_List cost_list;

	Command_and_Cost command_cost;
	Command command;
	Robot_State robot_state;
	double distance_interval, dist_to_rand, command_time, vel_reached;
	unsigned int initial_v_index, final_v_index;
	int phi_direction[2];

	distance_interval = GlobalState::distance_interval;
	dist_to_rand = near.distance(rand);
	if (dist_to_rand < GlobalState::distance_interval * 1.5)
		distance_interval = dist_to_rand;

	for (int i = 0; i < num_cost_function; i++)
	{
		cost_functions[i]->set_near(near.robot_state);
		cost_functions[i]->set_rand_pose(rand);
		cost_functions[i]->set_distance_interval(distance_interval);
	}


	get_velocity_vector_limits(near, distance_interval, initial_v_index, final_v_index, velocity_search_vector);

	phi_direction[0] = -1;
	phi_direction[1] = 1;

	//calculate costs, find min and max and put everyting into a vector
	for (unsigned int v_index = initial_v_index; v_index <= final_v_index; v_index++)
	{
		command.v = velocity_search_vector[v_index];

		command_time = get_travel_time(near.robot_state, command.v, distance_interval, &vel_reached);

		command.v = vel_reached;

		command_cost.command.v = command.v;
		command_cost.command_time = command_time;

		for (int i = 0; i < num_cost_function; i++)
		{
			cost_functions[i]->set_velocity(command.v);
			cost_functions[i]->set_time(command_time);
		}

		for(int phi_direction_index = 0; phi_direction_index < 2; phi_direction_index++)
		{
			for (unsigned int phi_mask_index = 0; phi_mask_index < phi_mask_vector.size(); phi_mask_index++)
			{
				command.phi = near.robot_state.v_and_phi.phi + phi_direction[phi_direction_index] * phi_mask_vector[phi_mask_index];

				if (!is_valid_phi(near.robot_state.v_and_phi.v, near.robot_state.v_and_phi.phi, command.v, command.phi, command_time))
					break;

				if (near.is_command_followed(command, command_time))
					continue;


				command_cost.command.phi = command.phi;

				robot_state = Ackerman::predict_next_pose_during_main_rrt_planning(near.robot_state, command, command_time);

				double state_cost = near.cost + command_time + command_time * Ackerman::get_env_cost(robot_state);

				//estimar custo para atingir goal e descartar caso necessario criar funcao
				if (GlobalState::goal_node &&
						(state_cost + (robot_state.distance(GlobalState::goal_node->robot_state.pose) / GlobalState::robot_config.max_vel))
						> GlobalState::goal_node->cost)
				{
					near.add_command(command_cost.command, command_cost.command_time);
					near.update_cvf();
					continue;
				}

				unsigned long int state_key = RRT_Node::get_key(robot_state);

				if (tree.nodes_map.find(state_key) != tree.nodes_map.end())
				{
					if (state_cost >= tree.nodes[tree.nodes_map[state_key]]->cost)
					{
						near.add_command(command_cost.command, command_cost.command_time);
						near.update_cvf();
						continue;
					}
				}


				command_cost.total_cost = 0.0;
				for (int i = 0; i < num_cost_function; i++)
				{
					cost_functions[i]->set_new_robot_state(robot_state);
					cost_functions[i]->set_phi(command.phi);

					command_cost.total_cost += cost_functions[i]->get_cost() * weights[i];
				}

				cost_list.push(command_cost);
			}
		}
	}

	int iter = 0;
	while (!cost_list.empty() && (iter < 5000))
	{
		iter++;

		command_cost = cost_list.top();

		if (Obstacle_Detection::is_obstacle_path(near, command_cost.command, command_cost.command_time))
		{
			near.add_command(command_cost.command, command_cost.command_time);
			near.update_cvf();
			cost_list.pop();
			continue;
		}

		*best_command = command_cost.command;
		*best_command_time = command_cost.command_time;
		*min_cost = command_cost.total_cost;
		new_node->robot_state = Ackerman::predict_next_pose_during_main_rrt_planning(near.robot_state, *best_command, *best_command_time);

		return true;
	}

	return false;
}


void
add_command_and_cost_to_x_near_list_of_approved_commands(RRT_Node &x_near, Robot_State robot_state, 
		Command_and_Cost command_and_cost, double &command_time,
		Tree &tree, Command_and_Cost_List &x_near_approved_commands_list)
{
	double state_cost = x_near.cost + command_time + command_time * Ackerman::get_env_cost(robot_state);

	if (GlobalState::goal_node &&
		(state_cost +
				(robot_state.distance(GlobalState::goal_node->robot_state.pose) / GlobalState::robot_config.max_vel)) >= GlobalState::goal_node->cost)
	{	// Adiciona o comando aa lista de comandos ja tentados e que devem ser descartados
		x_near.add_command(command_and_cost.command, command_and_cost.command_time);
		x_near.update_cvf();
		return;
	}

	unsigned long int state_key = RRT_Node::get_key(robot_state);
	if (tree.nodes_map.find(state_key) != tree.nodes_map.end())
	{	// Adiciona o comando aa lista de comandos ja tentados e que devem ser descartados
		if (state_cost >= tree.nodes[tree.nodes_map[state_key]]->cost)
		{
			x_near.add_command(command_and_cost.command, command_and_cost.command_time);
			x_near.update_cvf();
			return;
		}
	}
	x_near_approved_commands_list.push(command_and_cost);
}


void
try_to_add_a_command_with_this_phi_into_the_x_near_approved_commands_list(Command &command, double &command_time, RRT_Node &x_near,
		Cost_Function **cost_functions, const double *weights, const int num_cost_function,
		Command_and_Cost_List &x_near_approved_commands_list, Tree &tree, const double &best_cost_until_now)
{
	if ((command_time > 5.0) ||
//		((fabs(x_near.robot_state.v_and_phi.v) < 0.02) && (fabs(x_near.robot_state.v_and_phi.phi - command.phi)) < 0.05) ||
		!Ackerman::is_valid_phi(x_near.robot_state.v_and_phi.v, x_near.robot_state.v_and_phi.phi, command.v, command.phi, command_time) ||
		x_near.is_command_followed(command, command_time))
		return;

	// Eh na funcao abaixo que roda o modelo do carro, inclusive o atual modelo de giro do volante.
	// O comando command eh enviado ao robo por command_time segundos, mas seu estado pode nao chegar
	// ao especificado no command.
	Robot_State robot_state = Ackerman::predict_next_pose_during_main_rrt_planning(x_near.robot_state, command, command_time, NULL, 0.1);
	command.phi = robot_state.v_and_phi.phi; // phi alcancado pelo robot durante o tempo command_time

	Command_and_Cost command_and_cost = {{command.v, command.phi}, command_time, 0.0};
	for (int i = 0; i < num_cost_function; i++)
	{
		cost_functions[i]->set_new_robot_state(robot_state);
		cost_functions[i]->set_phi(command.phi);

		command_and_cost.total_cost += cost_functions[i]->get_cost() * weights[i];
	}

	if (command_and_cost.total_cost >= best_cost_until_now) // Alberto: Por que neste caso nao adicona aa lista de comandos descartados (dentro da funcao abaixo)?
		return;

	add_command_and_cost_to_x_near_list_of_approved_commands(x_near, robot_state, command_and_cost, command_time, tree, x_near_approved_commands_list);
}


void
compute_maximum_and_minimum_phi(double &maximum_phi, double &minimum_phi, RRT_Node& x_near, double command_time)
{
	double max_phi_change = carmen_get_phi_from_curvature(
			GlobalState::robot_config.desired_steering_command_rate * command_time, x_near.robot_state.v_and_phi.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	minimum_phi = carmen_clamp(-GlobalState::robot_config.max_phi,
			carmen_normalize_theta(x_near.robot_state.v_and_phi.phi - max_phi_change),
			GlobalState::robot_config.max_phi);

	maximum_phi = carmen_clamp(-GlobalState::robot_config.max_phi,
			carmen_normalize_theta(x_near.robot_state.v_and_phi.phi + max_phi_change),
			GlobalState::robot_config.max_phi);
}


void
try_to_add_a_command_with_this_velocity_into_the_x_near_approved_commands_list(Command_and_Cost_List &x_near_approved_commands_list,
		const double &velocity, RRT_Node &x_near,
		const double &distance_interval, Cost_Function **cost_functions, const double *weights, const int num_cost_function,
		const command_increment &phi_increment,
		Tree &tree, const double &best_cost_until_now)
{
	Command command;
	double vel_reached; // A funcao abaixo calcula o tempo de cada comando do rrt e a velocidade final apos cada comando
	double command_time = Ackerman::get_travel_time(x_near.robot_state, velocity, distance_interval, &vel_reached);
	command.v = vel_reached;

	double minimum_phi, maximum_phi;
	compute_maximum_and_minimum_phi(maximum_phi, minimum_phi, x_near, command_time);

	// Muda as cost_functions dependendo da velocidade
	int num_used_cost_functions = (fabs(GlobalState::last_odometry.v) < 1.5)? num_cost_function - 1: num_cost_function;
	double normalized_weights[20];
	if (num_used_cost_functions >= 20)
	{
		printf("Error: num_used_cost_functions >= 20 in try_to_add_a_command_with_this_velocity_into_the_x_near_approved_commands_list()\n");
		exit(1);
	}
	double sum_weights = 0.0;
	for (int i = 0; i < num_used_cost_functions; i++)
		sum_weights += weights[i];
	for (int i = 0; i < num_used_cost_functions; i++)
		normalized_weights[i] = weights[i] / sum_weights;

	for (int i = 0; i < num_used_cost_functions; i++)
	{
		cost_functions[i]->set_velocity(command.v);
		cost_functions[i]->set_time(command_time);
	}

	// manter phi
	command.phi = x_near.robot_state.v_and_phi.phi;
	try_to_add_a_command_with_this_phi_into_the_x_near_approved_commands_list(
			command, command_time, x_near, cost_functions, normalized_weights, num_used_cost_functions,
			x_near_approved_commands_list, tree, best_cost_until_now);

	// esquerda
	for (int i = 1; i <= phi_increment.max_i; i++)
	{
		double increment = phi_increment.get_increment_value(i);
		double phi = x_near.robot_state.v_and_phi.phi + increment;
		command.phi = carmen_clamp(minimum_phi, phi, maximum_phi);

		try_to_add_a_command_with_this_phi_into_the_x_near_approved_commands_list(
				command, command_time, x_near, cost_functions, normalized_weights, num_used_cost_functions,
				x_near_approved_commands_list, tree, best_cost_until_now);

		if (phi < minimum_phi || phi > maximum_phi)
			break;
	}

	// direita
	for (int i = 1; i <= phi_increment.max_i; i++)
	{
		double increment = phi_increment.get_increment_value(i);
		double phi = x_near.robot_state.v_and_phi.phi - increment;
		command.phi = carmen_clamp(minimum_phi, phi, maximum_phi);

		try_to_add_a_command_with_this_phi_into_the_x_near_approved_commands_list(
				command, command_time, x_near, cost_functions, normalized_weights, num_used_cost_functions,
				x_near_approved_commands_list, tree, best_cost_until_now);

		if (phi < minimum_phi || phi > maximum_phi)
			break;
	}

	// phi 0
	if (0 < minimum_phi || 0 > maximum_phi)
	{
		command.phi = 0.0;
		try_to_add_a_command_with_this_phi_into_the_x_near_approved_commands_list(
				command, command_time, x_near, cost_functions, normalized_weights, num_used_cost_functions,
				x_near_approved_commands_list, tree, best_cost_until_now);
	}
}


void
update_best_x_new_alternative_and_associated_command_and_time(
		RRT_Node *x_new, Command *x_new_associated_command, double *x_new_associated_time,
		RRT_Node &x_near, Command_and_Cost_List &x_near_approved_commands_list, double *min_cost)
{
	Command_and_Cost command_and_cost;

	int iter = 0;
	while (!x_near_approved_commands_list.empty() && (iter < 5000))
	{
		iter++;

		command_and_cost = x_near_approved_commands_list.top();

		if (command_and_cost.total_cost >= *min_cost)
			break; // Alberto: Esta condicao nao deveria fazer parte do if abaixo e, deste modo, permitir o descarte deste comando tentado?

		if (Obstacle_Detection::is_obstacle_path(x_near, command_and_cost.command, command_and_cost.command_time))
		{	// Adiciona o comando aa lista de comandos ja tentados e que devem ser descartados
			x_near.add_command(command_and_cost.command, command_and_cost.command_time);
			x_near.update_cvf();
			x_near_approved_commands_list.pop();
			continue;
		}

		// Computa um novo x_new completo, com o estado do robo e o comando que leva de x_near a x_new
		*x_new_associated_command = command_and_cost.command;
		*x_new_associated_time = command_and_cost.command_time;
		*min_cost = command_and_cost.total_cost;
		x_new->robot_state = Ackerman::predict_next_pose_during_main_rrt_planning(x_near.robot_state, *x_new_associated_command, *x_new_associated_time);

		break;
	}
}


double
get_current_max_velocity(double v, Robot_State &robot_state)
{
	double max_vel = v;
	double distance_to_start_stop = GlobalState::robot_config.max_vel * 3.0;

	if (GlobalState::last_goal)
	{
		double distance_to_goal = robot_state.pose.distance(*GlobalState::goal_pose);

		if (distance_to_goal < distance_to_start_stop)
		{
			double normalized_remaining_distance = distance_to_goal / distance_to_start_stop;
			max_vel = v * normalized_remaining_distance;
			robot_state.v_and_phi.v = max_vel;
		}
	}

	return max_vel;
}


bool
Ackerman::search_for_x_new_alternative_and_associated_command_and_time(
		RRT_Node *x_new_alternative, Command *x_new_alternative_associated_command, double *x_new_alternative_associated_time,
		RRT_Node &x_near, Pose &x_rand,
		Cost_Function **cost_functions, const double *weights, const int num_cost_function,
		double *min_cost, Tree &tree)
{
	command_increment vel_increment = get_command_increment(4.0, GlobalState::param_max_vel / 2.0, 10);
	command_increment phi_increment;
//	if (fabs(GlobalState::last_odometry.v) < 7.0)
		phi_increment = get_command_increment(4.0, GlobalState::robot_config.max_phi, 20);
//	else
//		phi_increment = get_command_increment(4.0 + (1.0 + 3.0 * (fabs(GlobalState::last_odometry.v) - 7.0)), GlobalState::robot_config.max_phi, 20);

	double distance_interval = GlobalState::distance_interval;
	distance_interval = fmin(GlobalState::distance_interval, x_near.distance(x_rand));

	double inferior_vel_limit, superior_vel_limit;
	get_travel_time(x_near.robot_state, -2.5, distance_interval, &inferior_vel_limit); // Alberto: este -2.5 esta esquisito...
	get_travel_time(x_near.robot_state, get_current_max_velocity(GlobalState::robot_config.max_vel, x_near.robot_state), distance_interval, &superior_vel_limit);

	for (int i = 0; i < num_cost_function; i++)
	{
		cost_functions[i]->set_near(x_near.robot_state);
		cost_functions[i]->set_rand_pose(x_rand);
		cost_functions[i]->set_distance_interval(distance_interval);
	}

	*min_cost = DBL_MAX;
	Command_and_Cost_List x_near_approved_commands_list;

	// manter velocidade se ela for suficientemente grande
	if (fabs(x_near.robot_state.v_and_phi.v) > 0.1)
	{
		double vel = x_near.robot_state.v_and_phi.v;

		try_to_add_a_command_with_this_velocity_into_the_x_near_approved_commands_list(x_near_approved_commands_list,
				vel, x_near,
				distance_interval, cost_functions, weights, num_cost_function, phi_increment, tree, *min_cost);

		update_best_x_new_alternative_and_associated_command_and_time(
				x_new_alternative, x_new_alternative_associated_command, x_new_alternative_associated_time,
				x_near, x_near_approved_commands_list, min_cost);
	}

	// aumentar velocidade
	for (int i = 1; i < vel_increment.max_i; i++)
	{
		double vel = carmen_clamp(inferior_vel_limit, x_near.robot_state.v_and_phi.v + vel_increment.get_increment_value(i), superior_vel_limit);
		if (fabs(vel) < 0.1)
			continue;

		try_to_add_a_command_with_this_velocity_into_the_x_near_approved_commands_list(x_near_approved_commands_list,
				vel, x_near,
				distance_interval, cost_functions, weights, num_cost_function, phi_increment, tree, *min_cost);

		update_best_x_new_alternative_and_associated_command_and_time(
				x_new_alternative, x_new_alternative_associated_command, x_new_alternative_associated_time,
				x_near, x_near_approved_commands_list, min_cost);
	}

	// diminuir velocidade
	for (int i = 1; i < vel_increment.max_i; i++)
	{
		double vel = carmen_clamp(inferior_vel_limit, x_near.robot_state.v_and_phi.v - vel_increment.get_increment_value(i), superior_vel_limit);
		if (fabs(vel) < 0.1)
			continue;

		try_to_add_a_command_with_this_velocity_into_the_x_near_approved_commands_list(x_near_approved_commands_list,
				vel, x_near,
				distance_interval, cost_functions, weights, num_cost_function, phi_increment, tree, *min_cost);

		update_best_x_new_alternative_and_associated_command_and_time(
				x_new_alternative, x_new_alternative_associated_command, x_new_alternative_associated_time,
				x_near, x_near_approved_commands_list, min_cost);
	}

	return *min_cost < DBL_MAX;
}

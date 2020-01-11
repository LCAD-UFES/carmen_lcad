/*
 * PathFollowerAckerman.cpp
 *
 *  Created on: 11/04/2012
 *      Author: romulo
 */

#include "path_follower_ackerman.h"
#include <math.h>
#include "../util/obstacle_detection.h"
#include "../util/publisher_util.h"
#include "../model/global_state.h"
#include "../util/ackerman.h"
#include "../util/util.h"
#include <carmen/navigator_gui_interface.h>

#define MOTION_COMMAND_VECTOR_MAX_SIZE 500
carmen_ackerman_motion_command_t motion_command_vector[MOTION_COMMAND_VECTOR_MAX_SIZE];

carmen_ackerman_motion_command_t path_motion_command[MOTION_COMMAND_VECTOR_MAX_SIZE];

int Path_Follower_Ackerman::use_obstacle_avoider = 0;

extern double t0;


void
Path_Follower_Ackerman::go()
{
	GlobalState::following_path = true;
}


void
Path_Follower_Ackerman::stop()
{
	GlobalState::following_path = false;
	// printf("stop\n");
	publish_path_follower_single_motion_command(0, 0, carmen_get_time());
}


void
Path_Follower_Ackerman::set_path(list<RRT_Path_Edge> path)
{
	if (!robot_lost && path.empty())
		return;

	this->path = path;

	if (!path.empty())
	{
		path_pose = path.begin()->p1;
		spent_time = 0.0;
		total_distance = -1;
		traveled_distance = 0.0;
	}

	robot_lost = false;
}


void
Path_Follower_Ackerman::set_path_lost_condition(double distance_threshold, double theta_threshold)
{
	this->distance_threshold = distance_threshold;
	this->theta_threshold = theta_threshold;
}


list<RRT_Path_Edge> &
Path_Follower_Ackerman::get_path()
{
	return path;
}


void
Path_Follower_Ackerman::update_path()
{
	double path_distance, theta_diff;

	if (path.empty() || !GlobalState::localize_pose)
		return;

	printf("entrou em Path_Follower_Ackerman::update_path() - %lf\n", carmen_get_time() - t0);

	// verificar se precisa mudar o comando
	while (Ackerman::remove_front_node_of_the_path(*GlobalState::localize_pose, path, path_distance, theta_diff,
				  &path_pose, &spent_time, &traveled_distance, &total_distance))
		;

	printf("passou do while em Path_Follower_Ackerman::update_path() - %lf\n", carmen_get_time() - t0);

	if (path_distance > distance_threshold || theta_diff > theta_threshold)
	{
		if (!robot_lost)
		{
			time_robot_lost = Util::get_time();
			robot_lost = true;
		}
		else if ((Util::get_time() - time_robot_lost) > 0.5) //se o robo se perdeu do caminho por mais que 0.5 segundos
		{
			path.clear();//stop robot
		}
	}
	else
	{
		robot_lost = false;
	}
}


void
Path_Follower_Ackerman::build_and_send_refined_path()
{
	if (GlobalState::current_algorithm != CARMEN_BEHAVIOR_SELECTOR_RRT ||
		!GlobalState::localize_pose ||
		!GlobalState::following_path)
		return;

	if (path.empty())
	{
//		if (GlobalState::last_odometry.v == 0.0)
//			publish_path_follower_single_motion_command(0.0, 0.0, carmen_get_time());
//		else
			publish_path_follower_single_motion_command(0.0, GlobalState::last_odometry.phi, carmen_get_time());
		return;
	}

	build_and_send_robot_motion_command_vector();
}


#ifdef OLD_STEERING_CONTROL

void
Path_Follower_Ackerman::compute_motion_command_vector(int &motion_command_size,
		Robot_State current_state, Command command, double remaining_command_time, double fixed_time)//, list<RRT_Path_Edge>::iterator edge)
{
	double command_time, command_remaining_dist, vel_reached, traveled_dist;

	//calcula a distancia do ponto atual em que esta o robo no caminho ate o fim do restante do comando
	Ackerman::predict_next_pose_during_main_rrt_planning(current_state, command, remaining_command_time, &command_remaining_dist);
	//divide a trajetoria em comandos menores de no maximo fixed_time segundos
	do
	{
		//retorna fixed_time ou algo menor (tempo para andar command_remaining_dist)
		command_time = fmin(Ackerman::get_travel_time(current_state, command.v, command_remaining_dist, &vel_reached), fixed_time);

		if (command_time <= 0.001)
			break;

		current_state = Ackerman::predict_next_pose_during_main_rrt_planning(current_state, command, command_time, &traveled_dist);

		command_remaining_dist -= traveled_dist;

		motion_command_vector[motion_command_size].v = current_state.v_and_phi.v;
		motion_command_vector[motion_command_size].phi = current_state.v_and_phi.phi;
		motion_command_vector[motion_command_size].time = command_time;
		motion_command_size++;

	} while ((command_remaining_dist > 0.0) && (motion_command_size < (MOTION_COMMAND_VECTOR_MAX_SIZE - 1)));
}

#else

void
Path_Follower_Ackerman::compute_motion_command_vector(int &motion_command_size,
		Robot_State current_state, Command requested_command, double full_time_interval, double delta_t,
		list<RRT_Path_Edge>::iterator edge)
{
	double max_phi_velocity = GlobalState::max_phi_velocity;
	double max_phi_acceleration = GlobalState::max_phi_acceleration;

	double phi_velocity = 0.0;
	// O codigo abaixo assume phi_velocity == 0.0 no início do full_time_interval
	double t_fim_descida;
	double t_fim_plato;
	double t_fim_subida;
	Ackerman::compute_intermediate_times(t_fim_subida, t_fim_plato, t_fim_descida, max_phi_acceleration, edge->p1.v_and_phi.phi, requested_command.phi,
			edge->time,	max_phi_velocity);

	int n = floor((edge->time - full_time_interval) / delta_t);
	double remaining_time = (edge->time - full_time_interval) - ((double) n * delta_t);
	Robot_State achieved_robot_state = edge->p1; // achieved_robot_state eh computado iterativamente abaixo a partir do estado dado do robo

	// Euler method -> avanca o robo ate o momento atual
	for (int i = 0; i < n; i++)
	{
		double t = (double) i * delta_t;
		Ackerman::predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, phi_velocity,
				t, delta_t,
				t_fim_descida, t_fim_plato, t_fim_subida,
				max_phi_velocity, max_phi_acceleration);
	}

	if (remaining_time > 0.0)
	{
		double t = (double) n * delta_t;
		Ackerman::predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, phi_velocity,
				t, remaining_time,
				t_fim_descida, t_fim_plato, t_fim_subida,
				max_phi_velocity, max_phi_acceleration);
	}

	n = floor(full_time_interval / delta_t);
	remaining_time = full_time_interval - ((double) n * delta_t);
	achieved_robot_state = current_state; // achieved_robot_state eh computado iterativamente abaixo a partir do estado dado do robo

	// Euler method
	for (int i = 0; (i < n) && (motion_command_size < (MOTION_COMMAND_VECTOR_MAX_SIZE - 1)); i++)
	{
		double t = (double) i * delta_t + (edge->time - full_time_interval);
		Ackerman::predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, phi_velocity,
				t, delta_t,
				t_fim_descida, t_fim_plato, t_fim_subida,
				max_phi_velocity, max_phi_acceleration);
		motion_command_vector[motion_command_size].v = achieved_robot_state.v_and_phi.v;
		motion_command_vector[motion_command_size].phi = achieved_robot_state.v_and_phi.phi;
		motion_command_vector[motion_command_size].time = delta_t;
		motion_command_size++;
	}

	if ((remaining_time > 0.0) && (motion_command_size < (MOTION_COMMAND_VECTOR_MAX_SIZE - 1)))
	{
		double t = (double) n * delta_t + (edge->time - full_time_interval);
		Ackerman::predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, phi_velocity,
				t, remaining_time,
				t_fim_descida, t_fim_plato, t_fim_subida,
				max_phi_velocity, max_phi_acceleration);
		motion_command_vector[motion_command_size].v = achieved_robot_state.v_and_phi.v;
		motion_command_vector[motion_command_size].phi = achieved_robot_state.v_and_phi.phi;
		motion_command_vector[motion_command_size].time = remaining_time;
		motion_command_size++;
	}

	//	if (fabs(achieved_robot_state.v_and_phi.v) > 0.07)
	//	{
	//		motion_command_vector[motion_command_size].v = achieved_robot_state.v_and_phi.v;
	//		motion_command_vector[motion_command_size].phi = achieved_robot_state.v_and_phi.phi;
	//		motion_command_vector[motion_command_size].time = delta_t;
	//		motion_command_size++;
	//	}
}
#endif


int
Path_Follower_Ackerman::handle_empty_rrt_path(int motion_command_size)
{
	if (GlobalState::last_path_received_is_empty)
	{
		double delta_t = GlobalState::localizer_pose_timestamp - GlobalState::last_rrt_path_message_timestamp;
		double max_delta_v = GlobalState::robot_config.maximum_deceleration_forward * delta_t / 1.5;
		for (int i = 0; i < motion_command_size; i++)
		{
			double previous_v = motion_command_vector[i].v;
			double new_v, reduction_factor;
			if (previous_v > 0.0)
			{
				new_v = carmen_clamp(0.0, previous_v - max_delta_v, previous_v);
				reduction_factor = new_v / previous_v;
			}
			else if (previous_v < 0.0)
			{
				new_v = carmen_clamp(previous_v, previous_v + max_delta_v, 0.0);
				reduction_factor = new_v / previous_v;
			}
			else
			{
				new_v = 0.0;
				reduction_factor = 1.0;
			}
			motion_command_vector[i].v = new_v;
			motion_command_vector[i].time *= 1.0 / reduction_factor;
			if (fabs(motion_command_vector[i].v) < 0.05)
			{
				motion_command_vector[i].time = 1.0;
				motion_command_vector[i].v = 0.0;
			}
		}
		// Cria um horizonte de comandos a frente para o obstacle_avoider atuar reduzindo a velocidade
		int last_motion_command = motion_command_size - 1;
		double last_motion_command_time = motion_command_vector[last_motion_command].time;
		double last_motion_command_v = motion_command_vector[last_motion_command].v;
		double last_motion_command_phi = motion_command_vector[last_motion_command].phi;
		double time_to_stop = 0.9 * (last_motion_command_v / GlobalState::robot_config.maximum_deceleration_forward); // v = at; t = v / a
		for (double t = 0.0; (t < time_to_stop) && (motion_command_size < (MOTION_COMMAND_VECTOR_MAX_SIZE - 1)); t += last_motion_command_time)
		{
			motion_command_vector[motion_command_size].v = last_motion_command_v;
			motion_command_vector[motion_command_size].phi = last_motion_command_phi;
			motion_command_vector[motion_command_size].time = last_motion_command_time;
			motion_command_size++;
		}
	}

	return (motion_command_size);
}


void
Path_Follower_Ackerman::build_and_send_robot_motion_command_vector()
{
	int motion_command_size = 0;
//	double fixed_time = 0.09;
	double fixed_time = 0.09 / 1.0;

	list<RRT_Path_Edge>::iterator edge = path.begin();

	printf("entrou em Path_Follower_Ackerman::build_and_send_robot_motion_command_vector() - %lf\n", carmen_get_time() - t0);

	 // path_pose é a posicao do caminho mais proxima da posicao do robo
	compute_motion_command_vector(motion_command_size, path_pose, edge->command, edge->time - spent_time, fixed_time);//, edge);
	edge++;

	while ((edge != path.end()) && (motion_command_size < (MOTION_COMMAND_VECTOR_MAX_SIZE - 1)))
	{
		compute_motion_command_vector(motion_command_size, edge->p1, edge->command, edge->time, fixed_time);//, edge);
		edge++;
	}

	motion_command_size = handle_empty_rrt_path(motion_command_size);

	publish_path_follower_motion_commands(motion_command_vector, motion_command_size, GlobalState::localizer_pose_timestamp);

	printf("saiu de Path_Follower_Ackerman::build_and_send_robot_motion_command_vector() - %lf\n", carmen_get_time() - t0);
}

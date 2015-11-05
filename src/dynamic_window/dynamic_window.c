/*
 * dynamic_window.c
 *
 *  Created on: 17/10/2011
 *      Author: rradaelli
 */
#include "dynamic_window.h"

void execute() {
	carmen_base_ackerman_velocity_message command = {0, 0, 0, NULL};
	carmen_base_ackerman_velocity_message best_command = {0, 0, 0, NULL};
	int count_vel_avaliadas = 0;
	int count_vel_indadmissiveis = 0;
	double initial_phi, initial_v;
	double final_phi, final_v;
	double v, phi;
	double max_g = 0;

	/*STOP CONDITIONS*/
	if(!is_target_set) {
		return;
	}

	if(carmen_distance(&pose, &target_pose) < 0.5) {
		send_command(0, 0);
		printf("Chegou ao objetivo\n");
		return;
	}

	/*DEFINING SEARCH SPACE*/
	//nao sabemos a aceleracao de rotacao da roda, por isso definimos o intervalo
	//entre +30 e -30 graus
	initial_phi = fmax(-max_phi, current_state.phi -phi_acceleration*interval_time);
	final_phi = fmin(max_phi, current_state.phi + 	phi_acceleration*interval_time);

	initial_v = fmax(0, current_state.v - 		deceleration*interval_time);
	final_v =  fmin(max_vel, current_state.v + 	acceleration*interval_time);

	/*SEARCHING*/
	for(v=initial_v; v<final_v; v+=0.01) {
		for(phi=initial_phi; phi<final_phi; phi+=carmen_degrees_to_radians(1)) {

			command.phi = phi;
			command.v = v;
			count_vel_avaliadas++;

			double d = dist(command)*max_distance;

			//verificar se a velocidade  admissivel
			if(command.v > sqrt(2*d*deceleration)
					/*|| (d*max_distance)<1 */) {//todo verificar
				//printf("Velocidade %.2f %.2f nao  admissivel\n", command.v, command.phi);
				count_vel_indadmissiveis++;
				continue;
			}

			double g = objective_function(command);

			if(g>max_g) {
				max_g = g;
				best_command = command;
			}
		}
	}

	printf("\nQnt Velocidade Avalidada: %d\nQnt Velocidade Inadimissivel: %d\n", count_vel_avaliadas, count_vel_indadmissiveis);
	printf("v: %.2f, phi: %.2f\n", best_command.v, carmen_radians_to_degrees(best_command.phi));

	double best_heading = heading(command);
	double best_dist = dist(command);
	double best_velocity = velocity(command);
	printf("heading: %f, dist: %f %f, velocity: %f\n", best_heading,
			best_dist, best_dist*max_distance, best_velocity);

	send_command(best_command.v, best_command.phi);
}

double objective_function(carmen_base_ackerman_velocity_message command) {
	return alfa*heading(command) + beta*dist(command) + gama*velocity(command);
}

double heading(carmen_base_ackerman_velocity_message command) {
	carmen_point_t predicted_pose;
	double target_theta;

	predicted_pose = predict_next_pose(pose, command, interval_time);

	target_theta = atan2(predicted_pose.y-target_pose.y, predicted_pose.x
			- target_pose.x) - predicted_pose.theta - M_PI;

	target_theta = carmen_normalize_theta(target_theta);

	return 1 - fabs(target_theta)/M_PI;//normaliza a saida entre 0 e 1
}

double dist(carmen_base_ackerman_velocity_message command) {
	carmen_point_t current = pose;
	carmen_point_t old = pose;
	double distance = 0;

	while(TRUE) {
		old = current;
		current = predict_next_pose(current, command, 0.5);

		double x = current.x / map.config.resolution;
		double y = current.y / map.config.resolution;

		if(!is_valid_position(x, y) || (fabs(pose.x-current.x) <0.005 &&
				fabs(pose.y-current.y) <0.005)) {
			return distance/max_distance;  //normalizar entre 0 e 1
		}

		distance += carmen_distance(&old, &current);

		if(distance>max_distance) {
			return 1;
		}

		if(is_obstacle(current)) {
			return distance/max_distance;
		}
	}
}

double velocity(carmen_base_ackerman_velocity_message command) {
	/*if(command.v<0) {
		return 0.005*(1-fabs(command.v)/max_vel);
	}*/

	return command.v/max_vel;
}

carmen_point_t predict_next_pose(carmen_point_t pose,
		carmen_base_ackerman_velocity_message command, double interval_time) {
	carmen_point_t new_pose = pose;

	new_pose.x +=  command.v * interval_time * cos(new_pose.theta);
	new_pose.y +=  command.v * interval_time * sin(new_pose.theta);
	new_pose.theta += carmen_normalize_theta(
			command.v*interval_time*tan(command.phi)/distance_between_front_and_rear_axles);

	new_pose.theta = carmen_normalize_theta(new_pose.theta);

	return new_pose;
}

int is_valid_position(int x, int y) {
	return x>=0 && x<map.config.x_size && y>=0 && y<map.config.y_size;
}

int is_obstacle_point(int x, int y) {
	double value;

	if(is_valid_position(x, y)) {
		value = map.map[(int)x][(int)y];

		return value>0.5;
	}

	return FALSE;
}

int is_obstacle(carmen_point_t pose) {
	double x = (pose.x/map.config.resolution);
	double y = (pose.y/map.config.resolution);

	double current_x = x;
	double current_y = y;

	double width_m = width/map.config.resolution;
	double height_m = height/map.config.resolution;

	double delta_x, delta_y;
	double alfa = M_PI/2 -pose.theta;

	for(double h=0; h<height_m; h+=0.5) {
		delta_x = cos(pose.theta) * h;
		delta_y = sin(pose.theta) * h;

		current_x = x+ delta_x;
		current_y = y+ delta_y;

		for(double w=0; w<width_m/2; w+=0.5) {
			int sinal = -1;

			delta_x = cos(alfa)*w;
			delta_y = sin(alfa)*w;

			for(int i=0; i<2; i++) {
				int aux_x, aux_y;

				aux_x = (int)(current_x + sinal*delta_x);
				sinal *=-1;
				aux_y = (int)(current_y + sinal*delta_y);

				if(is_obstacle_point(aux_x, aux_y)) {
					return TRUE;
				}
			}
		}
	}

	return FALSE;
}

void send_command(double v, double phi) {
	carmen_base_ackerman_velocity_message velocity_message;
	velocity_message.v = v;
	velocity_message.phi = phi;
	velocity_message.host = carmen_get_host();
	velocity_message.timestamp = carmen_get_time();

	IPC_publishData(CARMEN_BASE_ACKERMAN_VELOCITY_NAME, &velocity_message);
}

void print_map(int w) {
	int x = (int)(pose.x/map.config.resolution);
	int y = (int)(pose.y/map.config.resolution);

	for(int i=fmax(0, (x-w)); i<fmin(map.config.x_size, (x+w)); i++) {
		for(int j=fmax(0, (y-w)); j<fmin(map.config.y_size, (y+w)); j++) {
			printf("%2d",(int)(map.map[i][j]>0.5&&map.map[i][j]!=5&&map.map[i][j]!=4?1:map.map[i][j]));
		}
		printf("\n");
	}
}

/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#ifndef SIMULATOR_ACKERMAN_H
#define SIMULATOR_ACKERMAN_H

#define NUM_MOTION_COMMANDS_VECTORS	5
#define	NUM_MOTION_COMMANDS_PER_VECTOR	200

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double max_range;
  double variance;
  double prob_of_random_reading;
  double prob_of_random_max;
  int num_lasers;
  double offset;
  double side_offset;
  double angular_offset;
  double angular_resolution;
  double fov;
  double start_angle;
  int id;
} carmen_simulator_ackerman_laser_config_t;

typedef struct {
  int num_sonars;
  double max_range;
  double sensor_angle;
  carmen_point_t *offsets;
  double variance;
  double prob_of_random_reading;
  double prob_of_random_max;
} carmen_simulator_ackerman_sonar_config_t;


typedef struct 
{
	carmen_simulator_ackerman_laser_config_t front_laser_config;
	carmen_simulator_ackerman_laser_config_t rear_laser_config;
	carmen_simulator_ackerman_sonar_config_t sonar_config;
	int use_sonar;
	int use_front_laser, use_rear_laser;

	carmen_localize_ackerman_motion_model_t *motion_model;

	carmen_map_t map;
	carmen_point_t odom_pose;
	carmen_point_t true_pose;

	double v;
	double phi;
	double target_v;
	double target_phi;
	double width;
	double distance_between_rear_wheels;
	double distance_between_front_and_rear_axles;

	double delta_t;
	double real_time;
	double max_phi;
	double max_v;
	int sync_mode;
	double motion_timeout;
	double time_of_last_command;

	double maximum_steering_command_rate;
	double understeer_coeficient;
	double understeer_coeficient2;
	double maximum_speed_forward;
	double maximum_speed_reverse;
	double maximum_acceleration_forward;
	double maximum_deceleration_forward;
	double maximum_deceleration_reverse;
	double maximum_acceleration_reverse;
	double distance_between_rear_car_and_rear_wheels;
	double length;

	carmen_robot_and_trailer_motion_command_t *current_motion_command_vector;
	int nun_motion_commands;
	int current_motion_command_vector_index;

	carmen_localize_ackerman_globalpos_message global_pos;
	carmen_robot_ackerman_config_t robot_config;

	int initialize_neural_networks;
	unsigned int use_mpc;
	unsigned int use_rlpid;

} carmen_simulator_ackerman_config_t, *carmen_simulator_ackerman_config_p;


typedef struct {
	int exhaustive_value;
	double steering_output;
} exhaustive_value_to_steering_mapping;


typedef struct {
	double atan_desired_curvature;
	double atan_current_curvature;
	double b;
} state;


typedef struct {
	double steering_command;
} action;


#ifdef __cplusplus
}
#endif

#endif

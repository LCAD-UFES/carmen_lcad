/*
 * robot_config.h
 *
 *  Created on: 09/12/2011
 *      Author: rradaelli
 */

#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_

#include <carmen/carmen.h>

class Robot_Config
{
public:
	double length;
	double width;
	double max_vel;
	double max_phi;
	double understeer_coeficient;
	double distance_between_rear_wheels;
	double distance_between_front_and_rear_axles;
	double distance_between_rear_car_and_rear_wheels;
	double distance_between_front_car_and_front_wheels;
	double maximum_acceleration_forward;
	double maximum_deceleration_forward;
	double maximum_acceleration_reverse;
	double maximum_deceleration_reverse;
	double desired_steering_command_rate;
	double desired_acceleration;
	double desired_decelaration_forward;
	double desired_decelaration_reverse;
	double maximum_steering_command_rate;
};

#endif /* ROBOT_CONFIG_H_ */

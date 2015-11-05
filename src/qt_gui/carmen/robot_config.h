/*
 * robot_config.h
 *
 *  Created on: 09/12/2011
 *      Author: rradaelli
 */

#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_

class Robot_Config {
public:
	Robot_Config();
	virtual ~Robot_Config();

	double length;
	double width;
	double distance_between_rear_wheels;
	double distance_between_front_and_rear_axles;

};

#endif /* ROBOT_CONFIG_H_ */

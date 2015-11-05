/*
 * robot_state.h
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */
#ifndef ROBOT_STATE_H_
#define ROBOT_STATE_H_

#include "command.h"
#include "pose.h"

class Robot_State
{
public:
	double distance(Robot_State &r);
	double distance(Pose &pose);


	Pose	pose;
	Command v_and_phi;
};

#endif /* ROBOT_STATE_H_ */

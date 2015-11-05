/*
 * robot_state.cpp
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */

#include "robot_state.h"

double Robot_State::distance(Robot_State &r)
{
	return pose.distance(r.pose);
}

double Robot_State::distance(Pose &p)
{
	return pose.distance(p);
}

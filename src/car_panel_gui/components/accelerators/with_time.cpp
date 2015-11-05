/*
 * with_time.cpp
 *
 *  Created on: Apr 1, 2013
 *      Author: cayo
 */

#include "with_time.h"

withTime::withTime(double interval)
{
	this->interval = interval;
}

void
withTime::update(double velocity, double time)
{
	double acceleration = (velocity - this->velocity) / time;

	this->acceleration = (this->acceleration + acceleration) / 2;
	this->velocity = velocity;
	// intervalAccumulation += time;

	// this->acceleration = this->acceleration > maxRangeAcceleration ? this->acceleration - maxRangeAcceleration : this->acceleration < -maxRangeAcceleration ? this->acceleration + maxRangeAcceleration : this->acceleration;

	// if (intervalAccumulation >= interval)
	// {
	// 	if (velocity != this->velocity)
	// 	{
	// 		double acceleration = (velocity - this->velocity) / time;

	// 		this->acceleration = fabs(acceleration) > fabs(this->acceleration) ? acceleration : this->acceleration;
	// 		this->velocity = velocity;
	// 	}

	// 	intervalAccumulation = 0.0;
	// }
}

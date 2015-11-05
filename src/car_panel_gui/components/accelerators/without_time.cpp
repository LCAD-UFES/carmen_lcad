/*
 * without_time.cpp
 *
 *  Created on: Apr 1, 2013
 *      Author: cayo
 */

#include "without_time.h"

withoutTime::withoutTime(double interval)
{
	this->interval = interval;
	time = 0.0;
}

void
withoutTime::update(double velocity, double time)
{
	double acceleration = (velocity - this->velocity) / time;

	this->acceleration = (this->acceleration + acceleration) / 2;
	this->velocity = velocity;
	// if (this->time > 0.0)
	// {
	// 	double timeRange = time - this->time;

	// 	intervalAccumulation += timeRange;
	// 	this->acceleration = this->acceleration > maxRangeAcceleration ? this->acceleration - maxRangeAcceleration : this->acceleration < -maxRangeAcceleration ? this->acceleration + maxRangeAcceleration : this->acceleration;

	// 	if (intervalAccumulation >= interval)
	// 	{
	// 		if (velocity != this->velocity)
	// 		{
	// 			double acceleration = (velocity - this->velocity) / timeRange;

	// 			this->acceleration = fabs(acceleration) > fabs(this->acceleration) ? acceleration : this->acceleration;
	// 			this->velocity = velocity;
	// 		}

	// 		intervalAccumulation = 0.0;
	// 	}
	// }

	// this->time = time;
}

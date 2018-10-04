/*
 * without_time.cpp
 *
 *  Created on: Apr 1, 2013
 *      Author: cayo
 */

#include "../panel/without_time.h"

withoutTime::withoutTime()
{
	time = 0.0;
}

void
withoutTime::update(double velocity, double time)
{
	double new_acceleration;

	if (this->time != 0.0)
		new_acceleration = (velocity - this->velocity) / (time - this->time);
	else
		new_acceleration = 0.0;

	//acceleration = this->acceleration + (new_acceleration - this->acceleration) / 40.0; // Media para suavisar o display
	acceleration = new_acceleration;

	this->velocity = velocity;
	this->time = time;
}

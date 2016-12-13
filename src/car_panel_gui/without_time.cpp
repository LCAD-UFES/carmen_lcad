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
	double acceleration;

	if (this->time != 0.0)
		acceleration = (velocity - this->velocity) / (time - this->time);
	else
		acceleration = 0.0;

	this->acceleration = this->acceleration + (acceleration - this->acceleration) / 10.0; // Media para suavisar o display
	this->velocity = velocity;
	this->time = time;
}

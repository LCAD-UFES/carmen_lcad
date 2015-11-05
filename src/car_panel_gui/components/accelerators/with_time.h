/*
 * with_time.h
 *
 *  Created on: Apr 1, 2013
 *      Author: cayo
 */

#ifndef WITH_TIME_H_
#define WITH_TIME_H_

#include "../accelerator.h"

class withTime : public Accelerator
{
public:
	withTime(double interval);
	void update(double velocity, double time);
};

#endif /* WITH_TIME_H_ */

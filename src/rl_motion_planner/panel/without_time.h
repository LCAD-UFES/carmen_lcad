/*
 * without_time.h
 *
 *  Created on: Apr 1, 2013
 *      Author: cayo
 */

#ifndef WITHOUT_TIME_H_
#define WITHOUT_TIME_H_

#include "../panel/accelerator.h"

class withoutTime : public Accelerator
{
public:
	withoutTime();
	void update(double velocity, double time);

private:
	double time;
};

#endif /* WITHOUT_TIME_H_ */

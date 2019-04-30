
#ifndef __UTIL_TIME_H__
#define __UTIL_TIME_H__

#include <ctime>


class Timer
{
public:
	void start();
	double ellapsed();
	static double time();

protected:
	double _start;
};


#endif

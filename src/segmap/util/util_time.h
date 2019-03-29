
#ifndef __UTIL_TIME_H__
#define __UTIL_TIME_H__

#include <ctime>


class TimeCounter
{
public:
	void start();
	double ellapsed();
	static double time();

protected:
	double _start;
};


#endif

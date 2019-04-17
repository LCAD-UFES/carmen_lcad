
#include <ctime>
#include <sys/time.h>
#include <carmen/util_time.h>


void
Timer::start()
{
	_start = Timer::time();
}


double
Timer::time()
{
	double t;
	struct timeval tv;

	gettimeofday(&tv, NULL);
	t = tv.tv_sec + tv.tv_usec/1000000.0;

	return t;
}


double
Timer::ellapsed()
{
	return Timer::time() - _start;
}

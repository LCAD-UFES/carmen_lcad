
#include <ctime>
#include <sys/time.h>
#include <carmen/util_time.h>


void
TimeCounter::start()
{
	_start = TimeCounter::time();
}


double
TimeCounter::time()
{
	double t;
	struct timeval tv;

	gettimeofday(&tv, NULL);
	t = tv.tv_sec + tv.tv_usec/1000000.0;

	return t;
}


double
TimeCounter::ellapsed()
{
	return TimeCounter::time() - _start;
}

#include "Util.h"
#include <sys/time.h>
#include <cstdlib>


double
get_time()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);

	double time_in_sec = (tv.tv_sec) + ((double) tv.tv_usec * (double) 10e-7);
	return time_in_sec;
}


double
unitary_rand()
{
	return (double) rand() / (double) RAND_MAX;
}


double
gaussian_random(double mean, double std)
{
	const double norm = 1.0 / (RAND_MAX + 1.0);
	double u = 1.0 - rand() * norm;
	double v = rand() * norm;
	double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
	return mean + std * z;
}


void
fill_constant(double *vec, int n, int value)
{
	for (int i = 0; i < n; i++)
		vec[i] = value;
}

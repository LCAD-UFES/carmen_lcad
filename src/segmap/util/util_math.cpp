
#include <cmath>
#include <carmen/util_math.h>


double
normalize_theta(double theta)
{
	double multiplier;

	if (theta >= -M_PI && theta < M_PI)
		return theta;

	multiplier = floor(theta / (2*M_PI));
	theta = theta - multiplier*2*M_PI;
	if (theta >= M_PI)
		theta -= 2*M_PI;
	if (theta < -M_PI)
		theta += 2*M_PI;

	return theta;
}


int
argmax(double *v, int size)
{
	int p = 0;

	for (int i = 1; i < size; i++)
		if (v[i] > v[p])
			p = i;

	return p;
}


int
argmin(double *v, int size)
{
	int p = 0;

	for (int i = 1; i < size; i++)
		if (v[i] < v[p])
			p = i;

	return p;
}


double
mean(double *v, int size)
{
	double m = 0.;

	for (int i = 0; i < size; i++)
		m += v[i];

	if (size > 0)
		return m / (double) size;

	return 0.0;
}


int
argmax(std::vector<double> &v)
{
	return argmax(v.data(), v.size());
}


int
argmin(std::vector<double> &v)
{
	return argmin(v.data(), v.size());
}


double
mean(std::vector<double> &v)
{
	return mean(v.data(), v.size());
}


double
dist2d(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

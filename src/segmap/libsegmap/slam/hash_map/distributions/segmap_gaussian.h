
#ifndef __SEGMAP_GAUSSIAN_H__
#define __SEGMAP_GAUSSIAN_H__

#include <cmath>
#include "segmap_distribution_interface.h"


class Gaussian : public DistributionInterface<double>
{
public:

	// constexpr: https://www.geeksforgeeks.org/understanding-constexper-specifier-in-c/
	static constexpr double TINY_PROB = 1e-6;
	static constexpr double HIGH_PROB = 1.0 - TINY_PROB;
	static constexpr double LOG_TINY_PROB = log(TINY_PROB);
	static constexpr double LOG_HIGH_PROB = log(HIGH_PROB);
	static constexpr double LOG_ODDS_TINY_PROB = LOG_TINY_PROB / LOG_HIGH_PROB;
	static constexpr double LOG_ODDS_HIGH_PROB = LOG_HIGH_PROB / LOG_TINY_PROB;
	static constexpr double GAUSSIAN_MULTIPLIER = 1.0 / sqrt(2.0 * M_PI);
	static constexpr double LOG_GAUSSIAN_MULTIPLIER = log(GAUSSIAN_MULTIPLIER);

	double sum_squared, mean, std;
	long int n;

	Gaussian();
	virtual void update(const double &sample);
	virtual double likelihood(const double &sample);
	virtual double log_likelihood(const double &sample);
};


#endif

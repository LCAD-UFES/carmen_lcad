
#include "segmap_gaussian.h"


Gaussian::Gaussian() : DistributionInterface<double>()
{
	sum_squared = mean = std = 0.0;
	n = 0;
}


void
Gaussian::update(const double &sample)
{
	//SUM(i=1..n){values[i]^2} - period*(average^2)
	sum_squared += pow(sample, 2);
	mean = (sample + n * mean) / ((double) (n + 1));
	std = sqrt(sum_squared / (n + 1) - pow(mean, 2));
	n++;
}


double
Gaussian::likelihood(const double &sample)
{
	if (std > 0)
	{
		double multiplier = GAUSSIAN_MULTIPLIER * (1.0 / std);
		double exponent = -0.5 * pow((sample - mean) / std, 2);
		return multiplier * exp(exponent);
	}
	else
	{
		// if std = 0.0, then we have no uncertainty about the mean.
		if (sample == mean)
			return HIGH_PROB;
		else
			return TINY_PROB;
	}
}


double
Gaussian::log_likelihood(const double &sample)
{
	if (std > 0)
	{
		double exponent = -0.5 * pow((sample - mean) / std, 2);
		return LOG_GAUSSIAN_MULTIPLIER - log(std) + exponent;
	}
	else
	{
		// if std = 0.0, then we have no uncertainty about the mean.
		if (sample == mean)
			return LOG_HIGH_PROB;
		else
			return LOG_TINY_PROB;
	}
}



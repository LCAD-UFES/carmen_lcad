
#include "segmap_bernoulli.h"


Bernoulli::Bernoulli() : DistributionInterface<bool>()
{
	n_true = n_total = 0.0;
}


void
Bernoulli::update(const bool &sample)
{
	if (sample)
		n_true++;

	n_total++;
}


double
Bernoulli::likelihood(const bool &sample)
{
	double true_prob = n_true / n_total;

	if (sample)
		return true_prob;
	else
		return 1.0 - true_prob;
}


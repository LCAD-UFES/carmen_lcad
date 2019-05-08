
#ifndef __SEGMAP_BERNOULLI_H__
#define __SEGMAP_BERNOULLI_H__

#include "segmap_distribution_interface.h"


class Bernoulli : public DistributionInterface<bool>
{
public:
	double n_true, n_total;

	Bernoulli();
	virtual void update(const bool &sample);
	virtual double likelihood(const bool &sample);
	virtual double log_likelihood(const bool &sample);
};


#endif

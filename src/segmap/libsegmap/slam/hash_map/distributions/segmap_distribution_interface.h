
#ifndef __SEGMAP_DISTRIBUTION_INTERFACE_H__
#define __SEGMAP_DISTRIBUTION_INTERFACE_H__

#include <cmath>


template<class T>
class DistributionInterface
{
public:
	virtual ~DistributionInterface() { }
	virtual void update(const T &sample) = 0;
	virtual double likelihood(const T &sample) = 0;
	virtual double log_likelihood(const T &sample) { return log(likelihood(sample)); }

	// soft update
	// prior
	// kl-divergence
};


#endif

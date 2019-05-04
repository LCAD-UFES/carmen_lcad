
#ifndef __SEGMAP_CATEGORICAL_H__
#define __SEGMAP_CATEGORICAL_H__

#include <map>
#include "segmap_distribution_interface.h"


class Categorical : public DistributionInterface<int>
{
public:
	std::map<int, int> categories_count;
	int total;

	Categorical();

	virtual void update(const int &sample);
	virtual double likelihood(const int &sample);
	int most_likely();
};



#endif

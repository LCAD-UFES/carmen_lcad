
#include "segmap_categorical.h"


Categorical::Categorical() : DistributionInterface<int>()
{
	total = 0;
}


void
Categorical::update(const int &sample)
{
	std::map<int, int>::iterator it = categories_count.find(sample);

	if (it != categories_count.end())
		it->second++;
	else
		categories_count.insert(std::pair<int, int>(sample, 1));

	total++;
}


double
Categorical::likelihood(const int &sample)
{
	std::map<int, int>::iterator it = categories_count.find(sample);

	if (it != categories_count.end())
		return (double) it->second / (double) total;
	else
		return 0.0;
}


int
Categorical::most_likely()
{
	int n_max, id_max;
	std::map<int, int>::iterator it;

	id_max = n_max = -1;

	for (it = categories_count.begin(); it != categories_count.end(); it++)
	{
		if (it->second > n_max)
		{
			n_max = it->second;
			id_max = it->first;
		}
	}

	return id_max;
}


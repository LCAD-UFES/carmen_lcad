
#include <carmen/segmap_hash_map.h>
#include <cstdio>


int
main()
{
	pcl::PointXYZRGB point;
	HashGridMap<Bernoulli> map;

	point.r = 1;
	map.add(point);
	map.add(point);
	map.add(point);

	point.r = 0;
	map.add(point);

	printf("Prob+: %.3lf Prob-: %.3lf\n",
	       map.test_distr.likelihood(1),
	       map.test_distr.likelihood(0));

	return 0;
}


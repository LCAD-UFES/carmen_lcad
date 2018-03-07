
#include <cstdio>
#include "carmen_comm.h"

using namespace std;


void
print_vector(vector<float> v)
{
	for (unsigned int i = 0; i < v.size(); i++)
		printf("%.3f ", v[i]);

	printf("\n");
}


int
main()
{
	env_init();
	print_vector(env_reset(7757732.33, -363558.89, 0.651));

	while (!env_done())
		print_vector(env_step(1, 0));

	return 0;
}



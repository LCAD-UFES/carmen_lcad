
#include <cstdio>
#include <carmen/segmap_printers.h>

using namespace std;
using namespace Eigen;


// debug
void
print_vector(vector<double> &v)
{
	for (int i = 0; i < v.size(); i++)
		printf("%.2lf ", v[i]);

	printf("\n");
}


void
print_poses(vector<Matrix<double, 4, 4>> &poses)
{
	Matrix<double, 4, 4> p;

	for (int i = 0; i < poses.size(); i++)
	{
		p = poses[i];

		printf("%.2lf %.2lf %.2lf\n",
				p(0, 3) / p(3, 3),
				p(1, 3) / p(3, 3),
				p(2, 3) / p(3, 3));
	}
}

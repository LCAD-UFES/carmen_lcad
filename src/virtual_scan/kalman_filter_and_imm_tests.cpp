#include <carmen/carmen.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/velodyne_interface.h>
#include <carmen/matrix.h>
#include <iostream>
#include "virtual_scan.h"

using namespace std;


void
kalman_filter(Matrix &x_k, Matrix &P_k_k, Matrix z_k,
		Matrix F_k_1, Matrix Q_k_1, Matrix H_k, Matrix R_k)
{	// table 3.1 pg 23
	Matrix x_k_1_k_1 = x_k;
	Matrix P_k_1_k_1 = P_k_k;

	// Prediction
	Matrix x_k_k_1 = F_k_1 * x_k_1_k_1;
	Matrix P_k_k_1 = F_k_1 * P_k_1_k_1 * ~F_k_1 + Q_k_1;

	// Correction
	Matrix S_k = H_k * P_k_k_1 * ~H_k + R_k;
	Matrix K_k = P_k_k_1 * ~H_k * S_k.inv();
	x_k = x_k_k_1 + K_k * (z_k - H_k * x_k_k_1); // pequena mudancca aqui para evitar a necessidade da variavel ~zk
	P_k_k = P_k_k_1 - K_k * S_k * ~K_k;
}


int
main()
{
	double x, y, theta, v, delta_t;

	x = -170.0;
	y = -150.0;
	theta = carmen_degrees_to_radians(45.0);
	v = 5.0;
	delta_t = 1.0;

	double X[1 * 4] =
			   {x,
				v * cos(theta),
				y,
				v * sin(theta)};
	Matrix x_k(1, 4, X); // eq. 3.27, pg 24

	double P[4 * 4] =
	{
		10.0 * 10.0,	0.0, 			0.0, 			0.0,
		0.0, 			0.2*0.2, 		0.0, 			0.0,
		0.0, 			0.0, 			10.0 * 10.0,	0.0,
		0.0, 			0.0, 			0.0, 			0.2*0.2
	};
	Matrix P_k_k(4, 4, P); // eq. 3.33, pg 25
//	Matrix Q_k(1, 4);


	cout << x_k << endl;
	cout << P_k_k << endl;
//	for (int i = 0; i < 4; i++)
//		printf("i %d, m[1,i] %lf\n", i, x_k.val[0][i]);
//			Matrix &P_k_k, Matrix z_k,
//			Matrix F_k_1, Matrix Q_k_1, Matrix H_k, Matrix R_k
}

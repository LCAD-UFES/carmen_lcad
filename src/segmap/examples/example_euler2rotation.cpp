
#include <cmath>
#include <Eigen/Geometry>

using namespace Eigen;

int 
main()
{
    double rx, ry, rz;
    Matrix<double, 3, 3> Rx, Ry, Rz, R;

    // the inverse of this angle lead to an ambiguity.
    rx = 0.050175; // M_PI / 2.;
    ry = 0.003312; // M_PI / 6;
    rz = -0.9314506732051; // M_PI / 4.;

    Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
    Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
    Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;
    R  = Rz * Ry * Rx;

    /*
    R << 1., 5.55112e-17, 0., 
        -5.55112e-17, 1., 6.93889e-18, 
        -3.46945e-18, 0., 1.;
    */
    R = R.inverse() * R;    

    Matrix<double, 3, 1> ypr = R.eulerAngles(2, 1, 0);

	printf("rx: %lf ry: %lf rz: %lf\n", rx, ry, rz);
	printf("roll: %lf pitch: %lf yaw: %lf\n", ypr(2, 0), ypr(1, 0), ypr(0, 0));

    return 0;    
}

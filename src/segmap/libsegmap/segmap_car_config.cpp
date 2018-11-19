
#include "segmap_car_config.h"

#include <cmath>
#include <Eigen/Core>
#include <Eigen/LU>


using namespace Eigen;


Matrix<double, 3, 4>
kitti_velodyne_to_cam()
{
	Matrix<double, 4, 4> R00;
	Matrix<double, 3, 4> P02;
	Matrix<double, 4, 4> Rt;

	R00 << 9.999239e-01, 9.837760e-03, -7.445048e-03, 0.,
			-9.869795e-03, 9.999421e-01, -4.278459e-03, 0.,
			7.402527e-03, 4.351614e-03, 9.999631e-01, 0.,
			0., 0., 0., 1.;

	Rt << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
			1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
			9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
			0., 0., 0., 1.;

	P02 << 7.215377e+02, 0.000000e+00, 6.095593e+02, 4.485728e+01,
			0.000000e+00, 7.215377e+02, 1.728540e+02, 2.163791e-01,
			0.000000e+00, 0.000000e+00, 1.000000e+00, 2.745884e-03;

	return P02 * R00 * Rt;

}


Matrix<double, 3, 4>
carmen_velodyne_to_cam3(int image_width, int image_height)
{
	Matrix<double, 3, 4> projection;
	Matrix<double, 4, 4> velodyne2board;
	Matrix<double, 4, 4> cam2board;

	velodyne2board = pose6d_to_matrix(0.145, 0., 0.48, 0.0, -0.0227, -0.01);
	cam2board = pose6d_to_matrix(0.245, -0.04, 0.210, -0.017453, 0.026037, -0.023562);

	// This is a rotation to change the ref. frame from x: forward, y: left, z: up
	// to x: right, y: down, z: forward.
	Matrix<double, 4, 4> R;
	R = pose6d_to_matrix(0., 0., 0., 0., M_PI/2., -M_PI/2);
	//R = pose6d_to_matrix(0., 0., 0., 0., 0., 0.); // TODO

	double fx_factor = 0.764749;
	double fy_factor = 1.01966;
	double cu_factor = 0.505423;
	double cv_factor = 0.493814;
	double pixel_size = 0.00000375;

    double fx_meters = fx_factor * image_width * pixel_size;
    double fy_meters = fy_factor * image_height * pixel_size;

    double cu = cu_factor * image_width;
    double cv = cv_factor * image_height;

    // see http://www.cvlibs.net/publications/Geiger2013IJRR.pdf
    // Note: putting cu and cv in the 3rd column instead of the 4th is a trick
    // because to compute the actual pixel coordinates we divide the first two
    // dimensions of the point in homogenous coordinates by the third one (which is Z).
	projection << fx_meters / pixel_size, 0, cu, 0,
				  0, fy_meters / pixel_size, cv, 0,
				  0, 0, 1, 0.;

	return projection * R * cam2board.inverse() * velodyne2board;
}


Matrix<double, 4, 4>
carmen_vel2car()
{
	Matrix<double, 4, 4> velodyne2board;
	Matrix<double, 4, 4> board2car;

	velodyne2board = pose6d_to_matrix(0.145, 0., 0.48, 0.0, -0.0227, -0.01);
	board2car = pose6d_to_matrix(0.572, 0, 1.394, 0.0, 0.0122173048, 0.0);

	return board2car * velodyne2board;
}

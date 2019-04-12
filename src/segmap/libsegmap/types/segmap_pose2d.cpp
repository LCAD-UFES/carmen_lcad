

#include <cmath>
#include <Eigen/Core>
#include <carmen/segmap_pose2d.h>
#include <carmen/segmap_conversions.h>

using namespace Eigen;


Pose2d::Pose2d(double px, double py, double pth)
{
	x = px;
	y = py;
	th = pth;
}


Pose2d::Pose2d(const Pose2d &p)
{
	x = p.x;
	y = p.y;
	th = p.th;
}


Pose2d
Pose2d::operator=(const Pose2d &p)
{
	x = p.x;
	y = p.y;
	th = p.th;
	return *this;
}


double
Pose2d::theta_from_matrix(const Matrix<double, 4, 4> &m)
{
	// extract rotation matrix
	static Matrix<double, 3, 3> R;
	R << m(0, 0), m(0, 1), m(0, 2),
			m(1, 0), m(1, 1), m(1, 2),
			m(2, 0), m(2, 1), m(2, 2);

	// Important:
	// Extracting the yaw component from the rotation matrix is not
	// the right wayt of computing theta. Note that a rotation of yaw=pitch=roll=0
	// is equivalent to a rotation of yaw=pitch=roll=3.14 (in this order), but
	// yaw=0. is the opposite of yaw=3.14.
	// Matrix<double, 3, 1> ypr = R.eulerAngles(2, 1, 0);

	// Here I'm using the ad-hoc approach of rotating an unitary vector
	// and computing theta using the x and y coordinates. TODO: find a principled approach.
	static Matrix<double, 3, 1> unit;
	unit << 1, 0, 0;
	unit = R * unit;

	return atan2(unit(1, 0), unit(0, 0));
}


Pose2d
Pose2d::from_matrix(const Matrix<double, 4, 4> &m)
{
	Pose2d p;

	p.x = m(0, 3) / m(3, 3);
	p.y = m(1, 3) / m(3, 3);
	p.th = Pose2d::theta_from_matrix(m);

	return p;
}


Matrix<double, 4, 4>
Pose2d::to_matrix(const Pose2d &p)
{
	return pose6d_to_matrix(p.x, p.y, 0, 0, 0, p.th);
}


#include <cmath>
#include <vector>
#include <Eigen/LU>
#include <Eigen/Core>
#include <carmen/segmap_conversions.h>
#include <carmen/util_math.h>

using namespace Eigen;
using namespace std;


double
radians_to_degrees(double theta)
{
	return (theta * 180.0 / M_PI);
}


double
degrees_to_radians(double theta)
{
	return (theta * M_PI / 180.0);
}


Matrix<double, 4, 4>
pose6d_to_matrix(double x, double y, double z, double roll, double pitch, double yaw)
{
	Matrix<double, 3, 3> Rx, Ry, Rz, R;
	Matrix<double, 4, 4> T;

	double rx, ry, rz;

	rx = roll;
	ry = pitch;
	rz = yaw;

    Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
    Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
    Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;
    R  = Rz * Ry * Rx;

    T << R(0, 0), R(0, 1), R(0, 2), x,
    	R(1, 0), R(1, 1), R(1, 2), y,
		R(2, 0), R(2, 1), R(2, 2), z,
		0, 0, 0, 1;

    return T;
}


Matrix<double, 4, 4>
pose3d_to_matrix(double x, double y, double theta)
{
	return pose6d_to_matrix(x, y, 0., 0., 0., theta);
}


void
oxts2Mercartor(vector<vector<double>> &data, vector<Matrix<double, 4, 4>> &poses)
{
	double scale = cos(data[0][0] * M_PI / 180.);
	double er = 6378137;

	Matrix<double, 4, 4> p0;

	for (int i = 0; i < data.size(); i++)
	{
		double x, y, z;
		x = scale * data[i][1] * M_PI * er / 180;
		y = scale * er * log(tan((90 + data[i][0]) * M_PI / 360));
		z = data[i][2];

		double rx = data[i][3]; // roll
		double ry = data[i][4]; // pitch
		double rz = data[i][5]; // heading

		Matrix<double, 3, 3> Rx, Ry, Rz, R;
		Matrix<double, 4, 4> p;

		Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
		Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
		Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;
		R  = Rz * Ry * Rx;

		p << R(0, 0),  R(0, 1),  R(0, 2), x,
				R(1, 0),  R(1, 1),  R(1, 2), y,
				R(2, 0),  R(2, 1),  R(2, 2), z,
				0, 0, 0, 1;

		if (i == 0)
			p0 = p;

		p = p0.inverse() * p;
		poses.push_back(p);
	}
}


void
spherical2cartersian(double v_angle, double h_angle, double radius,
										 double *x, double *y, double *z)
{
	double cos_rot_angle = cos(h_angle);
	double sin_rot_angle = sin(h_angle);

	double cos_vert_angle = cos(v_angle);
	double sin_vert_angle = sin(v_angle);

	double xy_distance = radius * cos_vert_angle;

	*x = (xy_distance * cos_rot_angle);
	*y = (xy_distance * sin_rot_angle);
	*z = (radius * sin_vert_angle);
}


void
cartersian2spherical(double x, double y, double z,
										 double *v_angle, double *h_angle, double *radius)
{
	*radius = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

	if (*radius == 0)
	{
		*h_angle = *v_angle = 0;
	}
	else
	{
		*h_angle = normalize_theta(atan2(y, x));
		*v_angle = normalize_theta(asin(z / (*radius)));
	}
}


void
getEulerYPR(Matrix<double, 3, 3> &m_el, double &yaw, double &pitch, double &roll, unsigned int solution_number)
{
	struct Euler
	{
		double yaw;
		double pitch;
		double roll;
	};

	Euler euler_out;
	Euler euler_out2; //second solution
	//get the pointer to the raw data

	// Check that pitch is not at a singularity
	// Check that pitch is not at a singularity
	if (fabs(m_el(2, 0)) >= 1)
	{
		euler_out.yaw = 0;
		euler_out2.yaw = 0;

		// From difference of angles formula
		double delta = normalize_theta(atan2(m_el(2, 1), m_el(2, 2)));
		if (m_el(2, 0) < 0)  //gimbal locked down
		{
			euler_out.pitch = M_PI / double(2.0);
			euler_out2.pitch = M_PI / double(2.0);
			euler_out.roll = delta;
			euler_out2.roll = delta;
		}
		else // gimbal locked up
		{
			euler_out.pitch = -M_PI / double(2.0);
			euler_out2.pitch = -M_PI / double(2.0);
			euler_out.roll = delta;
			euler_out2.roll = delta;
		}
	}
	else
	{
		euler_out.pitch = -normalize_theta(asin(m_el(2, 0)));
		euler_out2.pitch = M_PI - euler_out.pitch;

		euler_out.roll = normalize_theta(atan2(m_el(2, 1)/cos(euler_out.pitch),
		                       m_el(2, 2)/cos(euler_out.pitch)));
		euler_out2.roll = normalize_theta(atan2(m_el(2, 1)/cos(euler_out2.pitch),
		                        m_el(2, 2)/cos(euler_out2.pitch)));

		euler_out.yaw = normalize_theta(atan2(m_el(1, 0)/cos(euler_out.pitch),
		                      m_el(0, 0)/cos(euler_out.pitch)));
		euler_out2.yaw = normalize_theta(atan2(m_el(1, 0)/cos(euler_out2.pitch),
		                       m_el(0, 0)/cos(euler_out2.pitch)));
	}

	if (solution_number == 1)
	{
		yaw = euler_out.yaw;
		pitch = euler_out.pitch;
		roll = euler_out.roll;
	}
	else
	{
		yaw = euler_out2.yaw;
		pitch = euler_out2.pitch;
		roll = euler_out2.roll;
	}
}

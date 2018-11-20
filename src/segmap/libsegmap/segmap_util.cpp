
#include "segmap_util.h"

#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>


using namespace std;
using namespace Eigen;


double
normalize_theta(double theta)
{
	double multiplier;

	if (theta >= -M_PI && theta < M_PI)
		return theta;

	multiplier = floor(theta / (2*M_PI));
	theta = theta - multiplier*2*M_PI;
	if (theta >= M_PI)
		theta -= 2*M_PI;
	if (theta < -M_PI)
		theta += 2*M_PI;

	return theta;
}


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


vector<Matrix<double, 4, 4>>
oxts2Mercartor(vector<vector<double>> &data)
{
	double scale = cos(data[0][0] * M_PI / 180.);
	double er = 6378137;

	// pose[i] contains the transformation which takes a
	// 3D point in the i'th frame and projects it into the oxts
	// coordinates of the first frame.
	vector<Matrix<double, 4, 4>> poses;
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

	return poses;
}


vector<double>
read_vector(char *name)
{
	double d;
	vector<double> data;

	FILE *f = fopen(name, "r");

	if (f == NULL)
		exit(printf("File '%s' not found.", name));

	while (!feof(f))
	{
		fscanf(f, "%lf\n", &d);
		data.push_back(d);
	}

	fclose(f);
	return data;
}


// debug
void
print_vector(vector<double> &v)
{
	for (int i = 0; i < v.size(); i++)
		printf("%.2lf ", v[i]);

	printf("\n");
}


int
argmax(double *v, int size)
{
	int p = 0;

	for (int i = 1; i < size; i++)
		if (v[i] > v[p])
			p = i;

	return p;
}


int
argmin(double *v, int size)
{
	int p = 0;

	for (int i = 1; i < size; i++)
		if (v[i] < v[p])
			p = i;

	return p;
}




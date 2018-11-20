
#ifndef _SEGMAP_UTIL_H_
#define _SEGMAP_UTIL_H_

#include <opencv/cv.h>
#include <vector>
#include <Eigen/Core>

using namespace cv;
using namespace std;
using namespace Eigen;


class CityScapesColorMap
{
	vector<Scalar> _colors;

public:
	int n_classes;

	CityScapesColorMap()
	{
		n_classes = 19;

		_colors.push_back(Scalar(128, 64, 128));
		_colors.push_back(Scalar(244, 35, 232));
		_colors.push_back(Scalar(70, 70, 70));
		_colors.push_back(Scalar(102, 102, 156));
		_colors.push_back(Scalar(190, 153, 153));
		_colors.push_back(Scalar(153, 153, 153));
		_colors.push_back(Scalar(250, 170, 30));
		_colors.push_back(Scalar(220, 220, 0));
		_colors.push_back(Scalar(107, 142, 35));
		_colors.push_back(Scalar(152, 251, 152));
		_colors.push_back(Scalar(70, 130, 180));
		_colors.push_back(Scalar(220, 20, 60));
		_colors.push_back(Scalar(255, 0, 0));
		_colors.push_back(Scalar(0, 0, 142));
		_colors.push_back(Scalar(0, 0, 70));
		_colors.push_back(Scalar(0, 60, 100));
		_colors.push_back(Scalar(0, 80, 100));
		_colors.push_back(Scalar(0, 0, 230));
		_colors.push_back(Scalar(119, 11, 32));
	}

	Scalar color(int index)
	{
		return _colors[index % _colors.size()];
	}
};


double normalize_theta(double theta);
double radians_to_degrees(double theta);
double degrees_to_radians(double theta);

Matrix<double, 4, 4> pose6d_to_matrix(double x, double y, double z, double roll, double pitch, double yaw);

vector<Matrix<double, 4, 4>> oxts2Mercartor(vector<vector<double>> &data);
vector<double> read_vector(char *name);

// debug
void print_vector(vector<double> &v);

int argmax(double *v, int size);
int argmin(double *v, int size);


#endif


#ifndef _SEGMAP_UTIL_H_
#define _SEGMAP_UTIL_H_

#include <vector>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

double normalize_theta(double theta);
double radians_to_degrees(double theta);
double degrees_to_radians(double theta);

Matrix<double, 4, 4> pose6d_to_matrix(double x, double y, double z, double roll, double pitch, double yaw);

vector<Matrix<double, 4, 4>> oxts2Mercartor(vector<vector<double>> &data);
vector<double> read_vector(char *name);

// debug
void print_vector(vector<double> &v);

#endif

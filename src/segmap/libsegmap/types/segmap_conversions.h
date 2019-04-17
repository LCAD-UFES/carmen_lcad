
#ifndef __SEGMAP_UTIL_CONVERSIONS_H__
#define __SEGMAP_UTIL_CONVERSIONS_H__

#include <Eigen/Core>
#include <vector>


double radians_to_degrees(double theta);
double degrees_to_radians(double theta);

Eigen::Matrix<double, 4, 4> pose6d_to_matrix(double x, double y, double z, double roll, double pitch, double yaw);
Eigen::Matrix<double, 4, 4> pose3d_to_matrix(double x, double y, double theta);

// pose[i] contains the transformation which takes a
// 3D pcl::Point in the i'th frame and projects it into the oxts
// coordinates of the first frame.
void oxts2Mercartor(std::vector<std::vector<double>> &data, std::vector<Eigen::Matrix<double, 4, 4>> &poses);

void spherical2cartersian(double v_angle, double h_angle, double radius,
                          double *x, double *y, double *z);

void cartersian2spherical(double x, double y, double z,
                          double *v_angle, double *h_angle, double *radius);

void getEulerYPR(Eigen::Matrix<double, 3, 3> &m_el,
                 double& yaw, double& pitch, double& roll,
                 unsigned int solution_number = 1);


#endif

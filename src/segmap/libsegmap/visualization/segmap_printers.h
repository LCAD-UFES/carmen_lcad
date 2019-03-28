
#ifndef __SEGMAP_PRINTERS_H__
#define __SEGMAP_PRINTERS_H__

#include <Eigen/Core>
#include <vector>


void print_vector(std::vector<double> &v);
void print_poses(std::vector<Eigen::Matrix<double, 4, 4>> &poses);


#endif

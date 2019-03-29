
#ifndef __SEGMAP_UTIL_MATH_H__
#define __SEGMAP_UTIL_MATH_H__

#include <vector>

double normalize_theta(double theta);

int argmax(double *v, int size);
int argmin(double *v, int size);
double mean(double *v, int size);

int argmax(std::vector<double> &v);
int argmin(std::vector<double> &v);
double mean(std::vector<double> &v);

double dist2d(double x1, double y1, double x2, double y2);

#endif

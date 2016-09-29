
#ifndef UTIL_H_
#define UTIL_H_

#include <cstdlib>
#include <cmath>
#include <vector>

using namespace std;

double get_time();
double unitary_rand();
double gaussian_random(double mean, double std);
void fill_constant(double *vec, int n, int value);

#endif

#include <math.h>

// GSL includes
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

#ifndef GSL_UTILS_H
#define GSL_UTILS_H

// C includes


double gsl_mahalanobis_distance(gsl_vector *x, gsl_vector *u, gsl_matrix *C);
double gsl_get_determinant(gsl_matrix *A);
double gsl_get_normal(gsl_vector *x, gsl_vector *u, gsl_matrix *cov);

#endif

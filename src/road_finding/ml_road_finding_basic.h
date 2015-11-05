#ifndef ML_ROAD_FINDING_BASIC_H
#define ML_ROAD_FINDING_BASIC_H

// C includes
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "time_profile.h"

// GSL includes
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

#define 	PI 3.141592653
#define 	SQRT_2_PI_POW_3 15.749610f

#define RED_INDEX 0
#define GREEN_INDEX 1
#define BLUE_INDEX 2

typedef struct
{
  int m; // number of pixels used to obtain this gaussian
  int m_t; // number of pixels used to obtain this gaussian decayed
  gsl_vector *mean; // the mean of the RGB components
  gsl_matrix *cov; // covariance matrix between RGB components
  double probability_threshould; // probability that is the threshould to accept a variable as fiting the Gaussian
  double probability_mean; // probability that represents the mean value
  double normalizer;

  unsigned long sums_of_products[3][3]; // matrix used to compute covariance
  unsigned long sums[3]; // vector used to compute covariance

  // data used to compute normal probability. keep it here and generate again only when gaussian changes, i.e, get more samples
  double det_cov;
  double cov_inv[9];
}rgb_gaussian;

typedef struct
{
  rgb_gaussian **global_gaussians;
  double threshould_distance;
  int n_max_gaussians;
  int n_gaussians;
  int m_width_step;
  int m_stereo_height;
} ml_road_finding;

typedef struct
{
  float *mean; // the mean of the RGB components
  float *cov; // covariance matrix between RGB components
  float *probability_threshould; // probability that is the threshould to accept a variable as fiting the Gaussian
  float *probability_mean; // probability that represents the mean value

  // data used to compute normal probability. keep it here and generate again only when gaussian changes, i.e, get more samples
  float *det_cov;
  float *cov_inv;
  float *normalizer;
}rgb_gaussian_gpu;

#endif

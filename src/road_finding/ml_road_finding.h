#ifndef ML_ROAD_FINDING_H
#define ML_ROAD_FINDING_H

#include "ml_road_finding_basic.h"

// OpenCV includes
#include <opencv/cv.h>
#include <opencv/highgui.h>

// OpenMP includes
#include <omp.h>

// My own includes
#include "gsl_utils.h"

using namespace std;

// Util debug methods
void print_matrix(gsl_matrix *m, FILE *dst CV_DEFAULT(stdout));
void print_gaussian(rgb_gaussian *gaussian, FILE *dst CV_DEFAULT(stdout));
void print_distances(ml_road_finding *road_finding_instance, rgb_gaussian *gaussian, FILE *dst CV_DEFAULT(stdout));

// Memory management methods
void init_ml_road_finding(ml_road_finding *road_finding_instance, int max_gaussians, int , int);
rgb_gaussian *init_rgb_gaussian(int n_samples);
void release_gaussian(rgb_gaussian *gaussian);
void clear_ml_road_finding(ml_road_finding *road_finding_instance);
void destroy_ml_road_finding(ml_road_finding *road_finding_instance);

// Collection methods
void add_gaussian(ml_road_finding *road_finding_instance, rgb_gaussian *new_gaussian);

// Extract Gaussian from samples
void find_ellipse(CvPoint *points, int n_points, CvPoint2D64f *center, CvPoint2D64f *axes);
int get_samples_at_rect(CvScalar *rgb_samples, IplImage *image, CvRect rect, int *mask);
rgb_gaussian *get_gaussian_at_rect(IplImage *image, CvRect rect, int *mask);
rgb_gaussian *get_gaussian_in_ellipse(IplImage *image, CvPoint2D64f C, CvPoint2D64f axes);
rgb_gaussian *get_gaussian_in_samples(CvScalar *rgb_samples, int n_samples);

void fill_image_based_on_gaussians(ml_road_finding *road_finding_instance, IplImage *src, IplImage *probabilistic_dst, int *mask);

#endif

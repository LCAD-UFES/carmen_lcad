#ifndef IMAGE_UTILS_H
#define IMAGE_UTILS_H

// C includes
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// OpenCV Includes
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>

using namespace std;

double image_utils_get_slope(CvPoint A, CvPoint B, int image_height);
CvPoint *image_utils_get_line_points(IplImage *img, CvPoint A, CvPoint B, int *n_points);
void image_utils_equalize(IplImage *src, IplImage *dst);
CvRect image_utils_get_rect(IplImage *image, CvPoint A, CvPoint B);

#endif

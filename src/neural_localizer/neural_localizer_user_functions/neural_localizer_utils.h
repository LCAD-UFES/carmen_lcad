#ifndef TRAFFIC_SIGN_UTILS_H_
#define TRAFFIC_SIGN_UTILS_H_

#include <opencv/cv.h>
#include <mae.h>

int
mae_int_random(int max);

double
mae_double_random(double max);

double
mae_uniform_random(double min, double max);

double
mae_gaussian_random(double mean, double std);

double
mae_normal_distribution(double std);

double
mae_radians_to_degrees(double theta);

double
mae_degrees_to_radians(double theta);

void
copy_image_to_neuron_layer(NEURON_LAYER *neuron_layer, CvRect *neuron_layer_roi, IplImage *image);

void
copy_neuron_layer_to_image(IplImage *image, CvRect *neuron_layer_roi, NEURON_LAYER *neuron_layer);

#endif /* TRAFFIC_SIGN_UTILS_H_ */

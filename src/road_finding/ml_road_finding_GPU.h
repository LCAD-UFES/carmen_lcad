#ifndef ML_ROAD_FINDING_GPU_H
#define ML_ROAD_FINDING_GPU_H

#include <cuda.h>


void init_cuda_road_finding(int n_gaussians, int widthStep, int height);

void add_gaussian_to_GPU(rgb_gaussian **gaussians, int n_gaussians);

void fill_image_based_on_gaussians_GPU(unsigned char *imageData, int width, int height, int widthStep, int n_gaussians);

#endif

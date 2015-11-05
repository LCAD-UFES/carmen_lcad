#ifndef CARMEN_LIB_V_DISPARITY_GPU_H
#define CARMEN_LIB_V_DISPARITY_GPU_H 1

#include <cuda.h>


void init_v_disparity_GPU(int width, int height, int nchannels);


void print_obstacles_GPU(unsigned char *dst_image, unsigned char *v_disparity_map, unsigned char *road_profile_image,
    float *disparity_map, float camera_height, float camera_pitch, float baseline, float focal_lenght, float xc, float yc, int stereo_width, int stereo_height, int stereo_disparity);

#endif

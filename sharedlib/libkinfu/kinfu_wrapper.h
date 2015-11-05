/*
 * kinfu_wrapper.h
 *
 *  Created on: Nov 12, 2012
 *      Author: lcad
 */

#ifndef KINFU_WRAPPER_H_
#define KINFU_WRAPPER_H_

bool
initialize_kinfu(int width, int height, float focal_length_x, float focal_lenght_y, float baseline, unsigned short max_depth, float volume_size, int gpu_device);


bool
execute_kinfu(double* position, double* rotation, double* orientation);

void
upload_depth_to_kinfu(unsigned short* depth);

#endif /* KINFU_WRAPPER_H_ */

#include <stdint.h>

#ifndef KINECT_UTIL_H_
#define KINECT_UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

float
convert_kinect_depth_raw_to_meters(uint16_t raw_depth);

uint16_t
convert_kinect_depth_meters_to_raw(float depth_in_meters);

float
convert_kinect_depth_mm_to_meters(uint16_t raw_depth);

char *
convert_double_to_string(double value);

#ifdef __cplusplus
}
#endif

#endif /* KINECT_UTIL_H_ */

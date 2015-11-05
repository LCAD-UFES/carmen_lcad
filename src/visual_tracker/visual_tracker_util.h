#ifndef CARMEN_VISUAL_TRACKER_UTIL_H
#define CARMEN_VISUAL_TRACKER_UTIL_H 1

#include <opencv/cv.h>

#ifdef __cplusplus
extern "C" {
#endif

void copy_RGB_image_to_BGR_image(unsigned char *original, IplImage *copy, int nchannels);
void copy_BGR_image_to_RGB_image(IplImage *original, unsigned char *copy, int nchannels);

#ifdef __cplusplus
}
#endif

#endif

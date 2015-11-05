/*********************************************************
 Stereo Camera Module, using libdc1394.
*********************************************************/

#ifndef CARMEN_STEREO_MESSAGES_H
#define CARMEN_STEREO_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int disparity_size;   		/* disparity_width * disparity_height */
  float *disparity;
  int reference_image_size;		/* input_image_width * input_image_height */
  unsigned char *reference_image;
  double timestamp;
  char *host;
}carmen_simple_stereo_disparity_message;

#define      CARMEN_SIMPLE_STEREO_DISPARITY_NAME       "carmen_simple_stereo_disparity"
#define      CARMEN_SIMPLE_STEREO_DISPARITY_FMT        "{int,<float:1>,int,<ubyte:3>,double,string}"

#ifdef __cplusplus
}
#endif

#endif



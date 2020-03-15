
#ifndef CARMEN_BUMBLEBEE_BASIC_MESSAGES_H
#define CARMEN_BUMBLEBEE_BASIC_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int width;                    /**<The x dimension of the image in pixels. */
  int height;                   /**<The y dimension of the image in pixels. */
  int image_size;              /**<width*height*bytes_per_pixel. */
  int isRectified;
  unsigned char *raw_left;
  unsigned char *raw_right;
  double timestamp;
  char *host;
} carmen_bumblebee_basic_stereoimage_message;

#define CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT "{int,int,int,int,<ubyte:3>,<ubyte:3>,double,string}"

#define CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_NAME "carmen_bumblebee_basic_stereoimage10"
#define CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_FMT "{int,int,int,int,<ubyte:3>,<ubyte:3>,double,string}"

#define CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE11_NAME "carmen_bumblebee_basic_stereoimage11"
#define CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE11_FMT "{int,int,int,int,<ubyte:3>,<ubyte:3>,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}

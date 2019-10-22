/*********************************************************
 Bumblebee2 Camera Module, using libdc1394.
*********************************************************/

#ifndef CARMEN_BUMBLEBEE_BASIC_MESSAGES_H
#define CARMEN_BUMBLEBEE_BASIC_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif


/*
*	Bumblebee Stereo Images Message
*/

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


#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_NAME       "carmen_bumblebe_basic_stereoimage"
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT        "{int,int,int,int,<ubyte:3>,<ubyte:3>,double,string}"

#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE1_NAME         "carmen_bumblebe_basic_stereoimage1"
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE2_NAME         "carmen_bumblebe_basic_stereoimage2"
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE3_NAME         "carmen_bumblebe_basic_stereoimage3"
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE4_NAME         "carmen_bumblebe_basic_stereoimage4"
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE5_NAME         "carmen_bumblebe_basic_stereoimage5"
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE6_NAME         "carmen_bumblebe_basic_stereoimage6"
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE7_NAME         "carmen_bumblebe_basic_stereoimage7"
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE8_NAME         "carmen_bumblebe_basic_stereoimage8"
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE9_NAME         "carmen_bumblebe_basic_stereoimage9"

#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE1_FMT     	CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE2_FMT     	CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE3_FMT     	CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE4_FMT     	CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE5_FMT     	CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE6_FMT     	CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE7_FMT     	CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE8_FMT     	CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT
#define      CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE9_FMT     	CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT

#ifdef __cplusplus
}
#endif

#endif

// @}

/*********************************************************
	 ---   My Module Specific Messages ---

See IPC documentation for more information:
http://www.cs.cmu.edu/~ipc/

*********************************************************/

#ifndef CARMEN_MINORU_MESSAGES_H
#define CARMEN_MINORU_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Message Struct Example */
typedef struct {
  int width;                    /**<The x dimension of the image in pixels. */
  int height;                   /**<The y dimension of the image in pixels. */
  int image_size;              /**<width*height*bytes_per_pixel. */
  int is_rectified;
  unsigned char *raw_left;
  unsigned char *raw_right;
  double timestamp; 		/* !!! obligatory !!! */
  char *host; 			/* !!! obligatory !!! */
} carmen_minoru_stereoimage_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_MINORU_STEREOIMAGE_NAME       "carmen_minoru_stereoimage"
#define 		 CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_NAME	"carmen_bumblebe_basic_stereoimage10"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_MINORU_STEREOIMAGE_FMT        "{int, int, int, int, <ubyte:3>, <ubyte:3>, double, string}"
#define 		 CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE10_FMT		"{int, int, int, int, <ubyte:3>, <ubyte:3>, double, string}"

#ifdef __cplusplus
}
#endif

#endif

// @}

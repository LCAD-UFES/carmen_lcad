#ifndef CARMEN_LASLAM_MESSAGES_H
#define CARMEN_LASLAM_MESSAGES_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
	int x, y;
} carmen_landmark_t;

typedef struct {
  int image_height;
  int image_width;
  int image_size;
  int landmark_list_size;
  unsigned char* reference_image;
  carmen_landmark_t* landmark_list;
  double timestamp;
  char *host;
} carmen_laslam_landmark_message;

#define      CARMEN_LASLAM_LANDMARK_MESSAGE_NAME       "carmen_laslam_landmark"
#define      CARMEN_LASLAM_LANDMARK_MESSAGE_FMT        "{int, int, int, int, <ubyte:3>, <{int, int}:4>, double, string}"

#ifdef __cplusplus
}
#endif

#endif

// @}

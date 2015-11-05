#ifndef CARMEN_NEURAL_GLOBAL_LOCALIZER_MESSAGES_H
#define CARMEN_NEURAL_GLOBAL_LOCALIZER_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  carmen_pose_3D_t pose;
  carmen_saliency_t saliencies[15];
  int image_size;
  int disparity_size;
  float* test_disparity;
  unsigned char* test_image;
  unsigned char* output_image;
  double confidence;
  double timestamp;
  char *host;
} carmen_neural_global_localizer_globalpos_message;

#define CARMEN_NEURAL_GLOBAL_LOCALIZER_GLOBALPOS_MESSAGE_NAME  "carmen_neural_global_localizer_globalpos"
#define CARMEN_NEURAL_GLOBAL_LOCALIZER_GLOBALPOS_MESSAGE_FMT   "{{{double,double,double},{double,double,double}},[{{int, int}, {double, double, double}}:15], int, int, <float:4>, <ubyte:3>, <ubyte:3>, double,double,string}"


#ifdef __cplusplus
}
#endif

#endif



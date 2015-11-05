#ifndef SALIENCY_SEARCH_MESSAGES_H_
#define SALIENCY_SEARCH_MESSAGES_H_

typedef struct {
  int image_height;
  int image_width;
  int image_size;
  unsigned char* reference_image;
  int saliency_list_size;
  carmen_saliency_t* saliency_list;
  carmen_pose_3D_t pose;
  double timestamp;
  char *host;
} carmen_saliency_search_saliency_points_message;

#define      CARMEN_SALIENCY_SEARCH_SALIENCY_POINTS_MESSAGE_NAME       "carmen_saliency_search_saliency_points"
#define      CARMEN_SALIENCY_SEARCH_SALIENCY_POINTS_MESSAGE_FMT        "{int, int, int, <ubyte:3>, int, <{int, int, ushort, float, [{double, double, double}:1]}:5>, {{double, double, double},{double, double, double}}, double, string}"

#endif /* SALIENCY_SEARCH_MESSAGES_H_ */

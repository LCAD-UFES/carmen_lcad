#ifndef CARMEN_STEREO_MAPPING_MESSAGES_H
#define CARMEN_STEREO_MAPPING_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int map_size;
  unsigned char *stereo_mapping_data;
  int measurement_size;
  float *measurement;
  double timestamp;
  char *host;
} carmen_stereo_mapping_message;

#define CARMEN_STEREO_MAPPING_FMT  "{int,<char:1>,int,<float:3>,double,string}"

#define CARMEN_STEREO_MAPPING_MSG_0_NAME "carmen_stereo_mapping_msg_0_0"
#define CARMEN_STEREO_MAPPING_MSG_1_NAME "carmen_stereo_mapping_msg_0_1"

#ifdef __cplusplus
}
#endif

#endif

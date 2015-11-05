#ifndef CARMEN_PROB_MESSAGES_H
#define CARMEN_PROB_MESSAGES_H

#include <carmen/global.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CARMEN_PROB_INITIALIZE_UNIFORM		1
#define CARMEN_PROB_INITIALIZE_GAUSSIAN	2

typedef struct {
	  int distribution_type;
	  int distribution_modes;
	  carmen_point_t *mean, *std;
	  double timestamp;
	  char *host;
} carmen_prob_initialize_message;
#define CARMEN_PROB_INITIALIZE_NAME "carmen_prob_initialize_msg"
#define CARMEN_PROB_INITIALIZE_FMT  "{int,int,<{double,double,double}:2>,<{double,double,double}:2>,double,string}"

#ifdef __cplusplus
}
#endif

#endif

#ifndef _VISUAL_SEARCH_USER_FUNCTIONS_H
#define _VISUAL_SEARCH_USER_FUNCTIONS_H

#include <carmen/carmen.h>
#include "filter.h"
#include "../neural_global_localizer.h"

#define DIRECTION_FORWARD	1
#define DIRECTION_REWIND	-1

#define MOVING_PHASE		0
#define TRAINING_PHASE		1
#define RECALL_PHASE		2

typedef struct {
     unsigned char red,green,blue;
} PPMPixel;

typedef struct {
     int x, y;
     PPMPixel *data;
} PPMImage;

typedef struct
{
	int frame;
	float confidence;
}winner_t;

typedef struct
{
	carmen_point_t coordinates;
	carmen_vector_3D_t pose;
}saliency_t;

typedef struct
{
	carmen_pose_3D_t pose;
	saliency_t saliencies[15];
	int image_id;
}frame_t;

typedef struct
{
	int camera;
	int training;
	int save_train;
}neural_global_localizer_config_t;

int g_frameID = -1;
int g_testedFrameID = 0;
frame_t* g_training_frame_list = NULL;

int g_has_new_frame = 0;
int g_networkStatus = TRAINING_PHASE;

#endif


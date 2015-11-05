#ifndef _TRAFFIC_SIGN_USER_FUNCTIONS_H
#define _TRAFFIC_SIGN_USER_FUNCTIONS_H

// Includes
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "mae.h"
#include "../neural_localizer.h"

// Macros
#define DIRECTION_FORWARD	1
#define DIRECTION_REWIND	-1

#define MOVING_PHASE		0
#define TRAINING_PHASE		1
#define RECALL_PHASE		2
// Types

struct _data_set
{
	char *input_name;
	char **input_sentences;
	int input_index;
	int num_input_sentences;
};

struct _image_set
{
	char image_name[256];
	int image_index;
};

typedef struct _data_set DATA_SET;
typedef struct _image_set IMAGE_SET;

// Prototypes

// Exportable Variables
extern int g_time_shift;

#endif /*_TRAFFIC_SIGN_USER_FUNCTIONS_H*/

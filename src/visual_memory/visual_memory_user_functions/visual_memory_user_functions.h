#ifndef _VISUAL_MEMORY_USER_FUNCTIONS_H
#define _VISUAL_MEMORY_USER_FUNCTIONS_H

// Includes
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "mae.h"
#include "../visual_memory.h"

//CARMEN
//CARMEN includes
#include <carmen/carmen.h>
#include <carmen/visual_memory_messages.h>
#include <carmen/visual_memory_interface.h>

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

typedef struct _data_set DATA_SET;

// Prototypes

// Exportable Variables
extern int g_time_shift;

#endif /*_TRAFFIC_SIGN_USER_FUNCTIONS_H*/

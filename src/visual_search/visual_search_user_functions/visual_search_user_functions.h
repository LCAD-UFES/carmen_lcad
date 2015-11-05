#ifndef _VISUAL_SEARCH_USER_FUNCTIONS_H
#define _VISUAL_SEARCH_USER_FUNCTIONS_H

// Includes
#include "filter.h"
#include "mae.h"	// due to train_neuron funtion usage

#ifdef USE_GRAYSCALE
#include "../visual_search_greyscale.h"
#else
#include "../visual_search.h"
#endif

//CARMEN includes
#include <carmen/carmen.h>
#include <carmen/visual_search_messages.h>
#include <carmen/visual_search_interface.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/bumblebee_basic_messages.h>

#define IDXPHOTO_MAX		13

#define DIRECTION_FORWARD	1
#define DIRECTION_REWIND	-1
#define POSE_MIN		1
#define POSE_MAX		26
#define MIN_PERSON_ID 		1
#define MAX_MAN_ID 		76
#define MAX_WOMAN_ID		60

#define MOVING_PHASE		0
#define TRAINING_PHASE		1
#define RECALL_PHASE		2

#define FILE_WIDTH		768
#define FILE_HEIGHT		576

#define IMAGE_FACTOR		FILE_WIDTH / IMAGE_WIDTH

#define MALE			0
#define FEMALE			1

#define EYE			0
#define NOSE			1
#define MOUTH			2
#define VIEW_LOG_POLAR		10

// Variaveis globais
int g_nStatus;
int g_nTries = 1;
int g_nTry = 1;

char g_strRandomFacesFileName[256];

float global_max_value = 0.0;

// External defined filtered pattern - for error data visualization - same in filter module
extern float	*gaussian_filtered_training_pattern;
extern float	gaussian_filtered_training_pattern_total_weight;

// Required Prototypes
inline void initialize_last_frame(void);
//inline void simple_raw_image_copy(unsigned char*,unsigned char*,int,int);
inline void copy_data_from_last_frame_to_neuron_layer(void);

void	set_input_layer_translation(INPUT_DESC*,int,int);
void	update_input_layer_neurons_and_image(INPUT_DESC*);
void	train_visual_search_app(void);
void	finalize_building_test_message_and_publish_it(INPUT_DESC*);
void	copy_raw_image_into_input(INPUT_DESC *,unsigned char *);
void	copy_raw_image_into_dst_image(unsigned char*,unsigned char*,int,int);
void	bumblebee_image_handler(carmen_bumblebee_basic_stereoimage_message *);
void	state_change_message_handler(carmen_visual_search_state_change_message *);
void	test_message_handler(carmen_visual_search_test_message *);
void	training_message_handler(carmen_visual_search_message *);
void 	query_message_handler(MSG_INSTANCE, BYTE_ARRAY, void *);
void	subscribe_bumblebee_basic_messages(int);

// Measuraments 
inline double	neuron_hamming_certainty (short int hamming_distance);

// CML prototypes
NEURON_OUTPUT SetNetworkStatus (PARAM_LIST *);
NEURON_OUTPUT SetNetworkStatus2 (PARAM_LIST *);
NEURON_OUTPUT GetRandomFace (PARAM_LIST *);
NEURON_OUTPUT update_input_filters(PARAM_LIST *);

NEURON_OUTPUT visual_search_converged(PARAM_LIST *pParamList);
NEURON_OUTPUT execute_IPC_listen(PARAM_LIST *);
NEURON_OUTPUT get_new_VS_state_change_message_value(PARAM_LIST *);
NEURON_OUTPUT get_new_VS_training_message_value(PARAM_LIST *);
NEURON_OUTPUT get_new_VS_test_message_value(PARAM_LIST *);
NEURON_OUTPUT get_visual_search_state(PARAM_LIST *);
NEURON_OUTPUT get_visual_search_state_message(PARAM_LIST *);
NEURON_OUTPUT reset_new_VS_state_change_message_value(PARAM_LIST *);
NEURON_OUTPUT reset_new_VS_training_message_value(PARAM_LIST *);
NEURON_OUTPUT reset_new_bumblebee_message_value(PARAM_LIST *);
NEURON_OUTPUT MAE_perform_state_change(PARAM_LIST *);
NEURON_OUTPUT MAE_perform_network_training(PARAM_LIST *);
NEURON_OUTPUT MAE_perform_network_test(PARAM_LIST *);
NEURON_OUTPUT MAE_perform_network_retraining(PARAM_LIST *);

#endif


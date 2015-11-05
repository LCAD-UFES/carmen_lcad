#ifndef _VISUAL_SEARCH_THIN_USER_FUNCTIONS_H
#define _VISUAL_SEARCH_THIN_USER_FUNCTIONS_H

// Includes
#include <mae.h>
#include <filter.h>
#include "../visual_search_thin.h"

//CARMEN includes
#include <carmen/carmen.h>
#include <carmen/visual_search_thin_messages.h>
#include <carmen/visual_search_thin_interface.h>

#ifdef __cplusplus
extern "C" {
#endif

// External defined filtered pattern - for error data visualization - same in filter module
extern float	*gaussian_filtered_training_pattern;
extern float	gaussian_filtered_training_pattern_sum;

// Required Prototypes
int		update_input_filters();
void	update_input_layer_neurons_and_image(INPUT_DESC*);
void 	update_input_layer_neurons_and_image_light(INPUT_DESC*);
void	set_input_layer_translation(INPUT_DESC *,int,int);

void	visual_search_thin_train(void);
void	visual_search_thin_saccade(INPUT_DESC *, int, float, int *, int *);
float	visual_search_thin_confidence(NEURON_LAYER *, int);
float	visual_search_thin_scale(NEURON_LAYER *);

#ifdef __cplusplus
}
#endif

#endif


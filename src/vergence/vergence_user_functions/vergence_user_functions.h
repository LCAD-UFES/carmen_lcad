#ifndef _VERGENCE_USER_FUNCTIONS_H
#define _VERGENCE_USER_FUNCTIONS_H

// Includes
#include <mae.h>
#include <filter.h>
#include "../vergence.h"

//CARMEN includes
#include <carmen/carmen.h>
#include <carmen/vergence_messages.h>
#include <carmen/vergence_interface.h>

#ifdef __cplusplus
extern "C" {
#endif

// External defined filtered pattern - for error data visualization - same in filter module
extern float	*gaussian_filtered_training_pattern;
extern float	gaussian_filtered_training_pattern_sum;

// Required Prototypes
int	update_input_filters();
void	update_input_layer_neurons_and_image(INPUT_DESC*);
void	set_input_layer_translation(INPUT_DESC *,int,int);

void	vergence_train(void);
void	vergence_train_positive(void);
void	vergence_saccade(INPUT_DESC *, int);
void	vergence_saccade_refined(INPUT_DESC *);
float	vergence_confidence(NEURON_LAYER *, int);
float	vergence_scale(NEURON_LAYER *);

#ifdef __cplusplus
}
#endif

#endif


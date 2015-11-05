#ifndef _TRACKER_USER_FUNCTIONS_H
#define _TRACKER_USER_FUNCTIONS_H

// Includes
#include <mae.h>
#include <filter.h>
#include "../tracker.h"

//CARMEN includes
#include <carmen/carmen.h>
#include <carmen/tracker_messages.h>
#include <carmen/tracker_interface.h>

#ifdef __cplusplus
extern "C" {
#endif

// External defined filtered pattern - for error data visualization - same in filter module
extern float *gaussian_filtered_training_pattern;
extern float gaussian_filtered_training_pattern_sum;

// Required Prototypes
int update_input_filters();
void update_input_layer_neurons_and_image(INPUT_DESC*);
void update_input_layer_neurons_and_image_light(INPUT_DESC*);
void set_input_layer_translation(INPUT_DESC *,int,int);

void tracker_train(void);
void tracker_saccade(INPUT_DESC *, int, float, int *, int *);
void visual_search(INPUT_DESC *);
float tracker_confidence();
float tracker_scale();

double compute_weigheted_neighborhood(NEURON *n, int w, int h, int u_neuron, int v_neuron, double log_factor);
int gt_x_displecement_from_fovea(int green);
int gt_y_displecement_from_fovea(int red);

#ifdef __cplusplus
}
#endif

#endif


// MAE 1.0 - THIS FILE WAS GERNERATED AUTOMATICALY

#ifndef _TRACKER_H
#define _TRACKER_H

// Includes
#include <stdio.h>
#include <stdlib.h>
#include "mae.h"
#include "filter.h"

// Definitions
#define TARGET_NEURON_LAYER_NAME ("nl_target_coordinates")
#define IMAGE_WIDTH (640)
#define IMAGE_HEIGHT (480)
#define BAND_WIDTH (0.125)
#define INPUTS_PER_NEURON (256)
#define GAUSSIAN_RADIUS (8.0)
#define LOG_FACTOR (2.0)
#define MAX_NUMBER_OF_SACCADE (4)
#define MIN_THRESHOLD_OF_SACCADE (0.5)
#define TAM_NL_ZOOM (1601)
#define IMAGE_WIDTH_RESIZED (201)
#define IMAGE_HEIGHT_RESIZED (201)

// Macros
#define NL_WIDTH (16 * 4 + 1)
#define NL_HEIGHT (12 * 4)

// Structs

// For avoiding symbol table errors on C++ linkage

#ifdef __cplusplus

// Prototypes
extern "C" void input_generator (INPUT_DESC *, int status);
extern "C" void input_controler (INPUT_DESC *, int status);
extern "C" void input_generator2 (INPUT_DESC *, int status);
extern "C" void input_controler2 (INPUT_DESC *, int status);
extern "C" void output_handler_weighted_average_value_position (OUTPUT_DESC *, int type_call, int mouse_button, int mouse_state); 
extern "C" void output_handler_resize (OUTPUT_DESC *, int type_call, int mouse_button, int mouse_state); 
extern "C" void translate_nl_filter (FILTER_DESC *);
extern "C" void translate_nl_filter_3 (FILTER_DESC *);
extern "C" void gaussian_filter (FILTER_DESC *);
extern "C" void red_mask_filter (FILTER_DESC *);
extern "C" void green_mask_filter (FILTER_DESC *);
extern "C" void blue_mask_filter (FILTER_DESC *);
extern "C" void map_image_v1 (FILTER_DESC *);
extern "C" void compute_weigheted_neighboor_filter (FILTER_DESC *);
extern "C" void generate_hough_activation_map (FILTER_DESC *);
extern "C" void generate_hough_zoom_activation_map (FILTER_DESC *);
extern "C" NEURON_TYPE minchinton;
extern "C" NEURON_OUTPUT run_train (PARAM_LIST *);
extern "C" NEURON_OUTPUT run_test (PARAM_LIST *);

// Global Variables
extern "C" int g_kernel_size;
extern "C" float translation_filter_deltaX;
extern "C" float translation_filter_deltaY;
extern "C" float translation_filter_deltaX_2;
extern "C" float translation_filter_deltaY_2;
extern "C" float dynamic_scale_factor;
extern "C" float dynamic_scale_factor_init;
extern "C" float g_halph_band_width;
extern "C" float HIGHEST_OUTPUT;
extern "C" float height_in_train;
extern "C" float width_in_train;
extern "C" float g_sigma;
extern "C" NEURON_LAYER nl_v1_activation_map;
extern "C" NEURON_LAYER nl_target_coordinates;
extern "C" NEURON_LAYER nl_v1_pattern;
extern "C" NEURON_LAYER in_saccade_translated;
extern "C" NEURON_LAYER in_saccade_translated_gaussian;
extern "C" NEURON_LAYER in_pattern_translated;
extern "C" NEURON_LAYER in_pattern_filtered_translated;
extern "C" NEURON_LAYER in_pattern_filtered_translated_red;
extern "C" NEURON_LAYER in_pattern_filtered_translated_green;
extern "C" NEURON_LAYER in_pattern_filtered_translated_blue;
extern "C" NEURON_LAYER table;
extern "C" NEURON_LAYER table_v1;
extern "C" NEURON_LAYER nl_v1_activation_map_neuron_weight;
extern "C" NEURON_LAYER nl_activation_map_hough;
extern "C" NEURON_LAYER nl_activation_map_hough_gaussian;
extern "C" NEURON_LAYER nl_activation_map_hough_v1;
extern "C" NEURON_LAYER nl_activation_map_hough_zoom;
extern "C" NEURON_LAYER nl_activation_map_hough_zoom_gaussian;
extern "C" INPUT_DESC in_saccade_current;
extern "C" INPUT_DESC in_saccade_trained;
extern "C" OUTPUT_DESC out_nl_v1_activation_map_neuron_weight;
extern "C" OUTPUT_DESC out_table;
extern "C" OUTPUT_DESC out_in_pattern_filtered_translated;
extern "C" OUTPUT_DESC out_nl_activation_map_hough_zoom;
extern "C" OUTPUT_DESC out_nl_activation_map_hough_zoom_gaussian;

#else

// Prototypes
extern void input_generator (INPUT_DESC *, int status);
extern void input_controler (INPUT_DESC *, int status);
extern void input_generator2 (INPUT_DESC *, int status);
extern void input_controler2 (INPUT_DESC *, int status);
extern void output_handler_weighted_average_value_position (OUTPUT_DESC *, int type_call, int mouse_button, int mouse_state); 
extern void output_handler_resize (OUTPUT_DESC *, int type_call, int mouse_button, int mouse_state); 
extern void translate_nl_filter (FILTER_DESC *);
extern void translate_nl_filter_3 (FILTER_DESC *);
extern void gaussian_filter (FILTER_DESC *);
extern void red_mask_filter (FILTER_DESC *);
extern void green_mask_filter (FILTER_DESC *);
extern void blue_mask_filter (FILTER_DESC *);
extern void map_image_v1 (FILTER_DESC *);
extern void compute_weigheted_neighboor_filter (FILTER_DESC *);
extern void generate_hough_activation_map (FILTER_DESC *);
extern void generate_hough_zoom_activation_map (FILTER_DESC *);
extern NEURON_TYPE minchinton;
extern NEURON_OUTPUT run_train (PARAM_LIST *);
extern NEURON_OUTPUT run_test (PARAM_LIST *);

// Global Variables
int g_kernel_size;
float translation_filter_deltaX;
float translation_filter_deltaY;
float translation_filter_deltaX_2;
float translation_filter_deltaY_2;
float dynamic_scale_factor;
float dynamic_scale_factor_init;
float g_halph_band_width;
float HIGHEST_OUTPUT;
float height_in_train;
float width_in_train;
float g_sigma;
NEURON_LAYER nl_v1_activation_map;
NEURON_LAYER nl_target_coordinates;
NEURON_LAYER nl_v1_pattern;
NEURON_LAYER in_saccade_translated;
NEURON_LAYER in_saccade_translated_gaussian;
NEURON_LAYER in_pattern_translated;
NEURON_LAYER in_pattern_filtered_translated;
NEURON_LAYER in_pattern_filtered_translated_red;
NEURON_LAYER in_pattern_filtered_translated_green;
NEURON_LAYER in_pattern_filtered_translated_blue;
NEURON_LAYER table;
NEURON_LAYER table_v1;
NEURON_LAYER nl_v1_activation_map_neuron_weight;
NEURON_LAYER nl_activation_map_hough;
NEURON_LAYER nl_activation_map_hough_gaussian;
NEURON_LAYER nl_activation_map_hough_v1;
NEURON_LAYER nl_activation_map_hough_zoom;
NEURON_LAYER nl_activation_map_hough_zoom_gaussian;
INPUT_DESC in_saccade_current;
INPUT_DESC in_saccade_trained;
OUTPUT_DESC out_nl_v1_activation_map_neuron_weight;
OUTPUT_DESC out_table;
OUTPUT_DESC out_in_pattern_filtered_translated;
OUTPUT_DESC out_nl_activation_map_hough_zoom;
OUTPUT_DESC out_nl_activation_map_hough_zoom_gaussian;

#endif

#endif

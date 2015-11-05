#ifndef _VISUAL_SEARCH_FILTERS_H
#define _VISUAL_SEARCH_FILTERS_H

// Includes
#include "filter.h"

#ifdef USE_GRAYSCALE
#include "../visual_search_greyscale.h"
#else
#include "../visual_search.h"
#endif

// Macros

// Types
struct	mask_point_list
{
	int	x;
	int	y;
	int	timer;
	struct	mask_point_list *next;
	struct	mask_point_list *previous;
};

typedef struct	mask_point_list MASK_POINT_LIST;

// Prototypes
void translate_nl_filter (FILTER_DESC *filter_desc);		// neural layer translation filter
void scale_nl_filter (FILTER_DESC *filter_desc);
void rotate_nl_filter (FILTER_DESC *filter_desc);
void nl_reshape_filter (FILTER_DESC *filter_desc);		// neural layer resizing filter
void scaled_map_image_v1 (FILTER_DESC *filter_desc);
void activation_pattern_overlay_filter (FILTER_DESC *filter_desc);

// General hamming certainty functions defined in user_functions module
inline double	hamming_squared_cosine_int(int hamming_dist, int ham_limit);
inline double	angular_similarity(double cosine);

// Mask point list functions
MASK_POINT_LIST* add_point_to_ihibition_list(MASK_POINT_LIST *list, int x, int y, int max_num_mov);
MASK_POINT_LIST* delete_node_from_ihibition_list(MASK_POINT_LIST *list, MASK_POINT_LIST *node);
int	is_xy_point_already_present_in_inhibition_list(MASK_POINT_LIST *list, int x, int y);
void	draw_inhibited_regions_in_nl_filter (FILTER_DESC *filter_desc);

// Error measure filters
void calculate_error_function_from_trained_output_filter (FILTER_DESC *filter_desc);
void output_layer_non_zero_activation_mean_value_threshold_filter (FILTER_DESC *filter_desc);
void output_layer_max_activation_value_percentage_threshold_filter (FILTER_DESC *filter_desc);

// External variables
extern float	*gaussian_filtered_training_pattern;
extern float	gaussian_filtered_training_pattern_total_weight;
extern MASK_POINT_LIST	*visual_search_mask_point_list;		// Extern mask point list
extern float	wisard_neuron_output_hash[];			// For hashing the wisard neuron output values

#endif

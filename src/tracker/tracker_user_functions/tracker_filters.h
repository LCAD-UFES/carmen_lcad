#ifndef _TRACKER_FILTERS_H
#define _TRACKER_FILTERS_H

// Includes
#include "filter.h"
#include "../tracker.h"

// Macros

// Types

// Prototypes
void gaussian_nl_filter(FILTER_DESC *filter_desc);
void translate_nl_filter (FILTER_DESC *filter_desc);		// neural layer translation filter
void scale_nl_filter (FILTER_DESC *filter_desc);
void rotate_nl_filter (FILTER_DESC *filter_desc);
void nl_reshape_filter (FILTER_DESC *filter_desc);		// neural layer resizing filter
void scaled_map_image_v1 (FILTER_DESC *filter_desc);


// External variables
extern float	*gaussian_filtered_training_pattern;
extern float	gaussian_filtered_training_pattern_sum;

#endif

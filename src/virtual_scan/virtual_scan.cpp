/*
 * virtual_scan.cpp
 *
 *  Created on: Jul 17, 2017
 *      Author: claudine
 */
#include <carmen/carmen.h>
#include "virtual_scan.h"


extended_virtual_scan_t *
virtual_scan_sort(carmen_mapper_virtual_scan_message *virtual_scan __attribute__ ((unused)))
{
	return (NULL);
}


virtual_scan_segment_t *
virtual_scan_segmentation(extended_virtual_scan_t *extended_virtual_scan __attribute__ ((unused)))
{
	return (NULL);
}


virtual_scan_box_model_t *
fit_box_models(virtual_scan_segment_t *virtual_scan_segments __attribute__ ((unused)))
{
	return (NULL);
}


void
detect_and_track_moving_objects(carmen_mapper_virtual_scan_message *virtual_scan __attribute__ ((unused)))
{
//	extended_virtual_scan_t *extended_virtual_scan = virtual_scan_sort(virtual_scan);
//	virtual_scan_segment_t *virtual_scan_segments = virtual_scan_segmentation(extended_virtual_scan);
//	virtual_scan_box_model_t *virtual_scan_box_models = fit_box_models(virtual_scan_segments);
//	neighborhood_graph_of_hypotheses = build_or_update_neighborhood_graph_of_hypotheses(virtual_scan_box_models);
//	moving_objects = sample_moving_objects_tracks(neighborhood_graph_of_hypotheses);
}

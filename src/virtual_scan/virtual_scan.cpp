/*
 * virtual_scan.cpp
 *
 *  Created on: Jul 17, 2017
 *      Author: claudine
 */
#include <carmen/carmen.h>
#include "virtual_scan.h"


extended_virtual_scan_t extended_virtual_scan;
carmen_point_t extended_virtual_scan_points[10000];


int
compare_angles(const void *a, const void *b)
{
	extended_virtual_scan_t arg1 = *((extended_virtual_scan_t *) a);
	extended_virtual_scan_t arg2 = *((extended_virtual_scan_t *) b);

	// The contents of the array are being sorted in ascending order
	if (arg1.point->theta < arg2.point->theta)
		return -1;
	if (arg1.point->theta > arg2.point->theta)
		return 1;
	return 0;
}


extended_virtual_scan_t *
virtual_scan_sort(carmen_mapper_virtual_scan_message *virtual_scan)
{
	extended_virtual_scan.point = extended_virtual_scan_points;
	for (int i = 0; i < virtual_scan->num_points; i++)
	{
		extended_virtual_scan.point[i].x = virtual_scan->points[i].x;
		extended_virtual_scan.point[i].y = virtual_scan->points[i].y;
		double theta = atan2(virtual_scan->points[i].y - virtual_scan->globalpos.y, virtual_scan->points[i].x - virtual_scan->globalpos.x);
		theta = carmen_normalize_theta(theta - virtual_scan->globalpos.theta);
		extended_virtual_scan.point[i].theta = theta;
	}
	qsort((void *) (extended_virtual_scan.point), (size_t) extended_virtual_scan.num_points, sizeof(carmen_point_t), compare_angles);

//	for (int i = 0; i < extended_virtual_scan.num_points; i++)
//	{
//		printf("%f", extended_virtual_scan.point[i].theta);
//	}

	return (&extended_virtual_scan);
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
detect_and_track_moving_objects(carmen_mapper_virtual_scan_message *virtual_scan)
{
	__attribute__ ((unused)) extended_virtual_scan_t *extended_virtual_scan = virtual_scan_sort(virtual_scan);
//	virtual_scan_segment_t *virtual_scan_segments = virtual_scan_segmentation(extended_virtual_scan);
//	virtual_scan_box_model_t *virtual_scan_box_models = fit_box_models(virtual_scan_segments);
//	neighborhood_graph_of_hypotheses = build_or_update_neighborhood_graph_of_hypotheses(virtual_scan_box_models);
//	moving_objects = sample_moving_objects_tracks(neighborhood_graph_of_hypotheses);
}

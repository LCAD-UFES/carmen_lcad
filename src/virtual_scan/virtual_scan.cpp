/*
 * virtual_scan.cpp
 *
 *  Created on: Jul 17, 2017
 *      Author: claudine
 */
#include <carmen/carmen.h>
#include <string.h>
#include "virtual_scan.h"


extended_virtual_scan_t extended_virtual_scan;
carmen_point_t extended_virtual_scan_points[10000];


int
compare_angles(const void *a, const void *b)
{
	carmen_point_t *arg1 = (carmen_point_t *) a;
	carmen_point_t *arg2 = (carmen_point_t *) b;

	// The contents of the array are being sorted in ascending order
	if (arg1->theta < arg2->theta)
		return -1;
	if (arg1->theta > arg2->theta)
		return 1;
	return 0;
}


extended_virtual_scan_t *
virtual_scan_sort(carmen_mapper_virtual_scan_message *virtual_scan)
{
	extended_virtual_scan.point = extended_virtual_scan_points;
	extended_virtual_scan.num_points = virtual_scan->num_points;
	for (int i = 0; i < virtual_scan->num_points; i++)
	{
		extended_virtual_scan.point[i].x = virtual_scan->points[i].x;
		extended_virtual_scan.point[i].y = virtual_scan->points[i].y;
		double theta = atan2(virtual_scan->points[i].y - virtual_scan->globalpos.y, virtual_scan->points[i].x - virtual_scan->globalpos.x);
		theta = carmen_normalize_theta(theta - virtual_scan->globalpos.theta);
		extended_virtual_scan.point[i].theta = theta;
	}
	qsort((void *) (extended_virtual_scan.point), (size_t) extended_virtual_scan.num_points, sizeof(carmen_point_t), compare_angles);

	return (&extended_virtual_scan);
}


virtual_scan_segments_t *
virtual_scan_segmentation(extended_virtual_scan_t *extended_virtual_scan)
{
	int segment_id = 0;
	int begin_segment = 0;
	virtual_scan_segments_t *virtual_scan_segments;

	virtual_scan_segments = (virtual_scan_segments_t *) malloc(sizeof(virtual_scan_segments_t));

	virtual_scan_segments->num_segments = 0;
	virtual_scan_segments->segment = NULL;
	for (int i = 1; i < extended_virtual_scan->num_points; i++)
	{
		if ((DIST2D(extended_virtual_scan->point[i],  extended_virtual_scan->point[i - 1]) > 0.6) ||
			(i == extended_virtual_scan->num_points - 1))
		{
			virtual_scan_segments->segment = (virtual_scan_segment_t *) realloc(virtual_scan_segments->segment,
					sizeof(virtual_scan_segment_t) * (segment_id + 1));
			if (i < extended_virtual_scan->num_points - 1)
				virtual_scan_segments->segment[segment_id].num_points = i - begin_segment;
			else
				virtual_scan_segments->segment[segment_id].num_points = i - begin_segment + 1;
			virtual_scan_segments->segment[segment_id].point = &(extended_virtual_scan->point[begin_segment]);
			begin_segment = i;
			segment_id++;
			virtual_scan_segments->num_segments++;
		}
	}

	return (virtual_scan_segments);
}


virtual_scan_box_model_t *
fit_box_models(virtual_scan_segment_t *virtual_scan_segments __attribute__ ((unused)))
{
	return (NULL);
}


extended_virtual_scan_t *
detect_and_track_moving_objects(carmen_mapper_virtual_scan_message *virtual_scan)
{
	extended_virtual_scan_t *extended_virtual_scan = virtual_scan_sort(virtual_scan);
//	virtual_scan_segment_t *virtual_scan_segments = virtual_scan_segmentation(extended_virtual_scan);
//	virtual_scan_box_model_t *virtual_scan_box_models = fit_box_models(virtual_scan_segments);
//	neighborhood_graph_of_hypotheses = build_or_update_neighborhood_graph_of_hypotheses(virtual_scan_box_models);
//	moving_objects = sample_moving_objects_tracks(neighborhood_graph_of_hypotheses);
	return (extended_virtual_scan);
}

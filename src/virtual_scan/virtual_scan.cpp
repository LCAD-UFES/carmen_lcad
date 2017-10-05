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
sort_virtual_scan(carmen_mapper_virtual_scan_message *virtual_scan)
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
segment_virtual_scan(extended_virtual_scan_t *extended_virtual_scan)
{
	int segment_id = 0;
	int begin_segment = 0;
	virtual_scan_segments_t *virtual_scan_segments;

	virtual_scan_segments = (virtual_scan_segments_t *) malloc(sizeof(virtual_scan_segments_t));
	virtual_scan_segments->num_segments = 0;
	virtual_scan_segments->segment = NULL;
	for (int i = 1; i < extended_virtual_scan->num_points; i++)
	{
		if ((DIST2D(extended_virtual_scan->point[i],  extended_virtual_scan->point[i - 1]) > 1.5) ||
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


static double
dist2(carmen_point_t v, carmen_point_t w)
{
	return (carmen_square(v.x - w.x) + carmen_square(v.y - w.y));
}


carmen_point_t
distance_from_point_to_line_segment_vw(int *point_in_trajectory_is, carmen_point_t v, carmen_point_t w, carmen_point_t p)
{

#define	POINT_WITHIN_SEGMENT		0
#define	SEGMENT_TOO_SHORT			1
#define	POINT_BEFORE_SEGMENT		2
#define	POINT_AFTER_SEGMENT			3

	// Return minimum distance between line segment vw and point p
	// http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
	double l2, t;

	l2 = dist2(v, w); // i.e. |w-v|^2 // NAO TROQUE POR carmen_ackerman_traj_distance2(&v, &w) pois nao sei se ee a mesma coisa.
	if (l2 < 0.1)	  // v ~== w case // @@@ Alberto: Checar isso
	{
		*point_in_trajectory_is = SEGMENT_TOO_SHORT;
		return (v);
	}

	// Consider the line extending the segment, parameterized as v + t (w - v).
	// We find the projection of point p onto the line.
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;

	if (t < 0.0) 	// p beyond the v end of the segment
	{
		*point_in_trajectory_is = POINT_BEFORE_SEGMENT;
		return (v);
	}
	if (t > 1.0)	// p beyond the w end of the segment
	{
		*point_in_trajectory_is = POINT_AFTER_SEGMENT;
		return (w);
	}

	// Projection falls on the segment
	p = v; // Copy other elements, like theta, etc.
	p.x = v.x + t * (w.x - v.x);
	p.y = v.y + t * (w.y - v.y);
	*point_in_trajectory_is = POINT_WITHIN_SEGMENT;

	return (p);
}


carmen_point_t
compute_segment_centroid(virtual_scan_segment_t virtual_scan_segment)
{
	carmen_point_t centroid;

	for (int i = 0; i < virtual_scan_segment.num_points; i++)
	{
		centroid.x += virtual_scan_segment.point[i].x;
		centroid.y += virtual_scan_segment.point[i].y;
	}
	centroid.x /= virtual_scan_segment.num_points;
	centroid.y /= virtual_scan_segment.num_points;

	return(centroid);
}


virtual_scan_segment_classes_t *
classify_segments(virtual_scan_segments_t *virtual_scan_segments)
{
	virtual_scan_segment_classes_t *virtual_scan_segment_classes;
	virtual_scan_segment_classes = (virtual_scan_segment_classes_t *) malloc(sizeof(virtual_scan_segment_classes_t));
	int num_segments = virtual_scan_segments->num_segments;
	virtual_scan_segment_classes->num_segments = num_segments;
	virtual_scan_segment_classes->segment = (virtual_scan_segment_t *) malloc(sizeof(virtual_scan_segment_t)*num_segments);
	virtual_scan_segment_classes->segment_features = (virtual_scan_segment_features_t *) malloc(sizeof(virtual_scan_segment_features_t)*num_segments);

	for (int i = 0; i < virtual_scan_segments->num_segments; i++)
	{
		int num_points = virtual_scan_segments->segment[i].num_points;
		carmen_point_t v = virtual_scan_segments->segment[i].point[0];
		carmen_point_t w = virtual_scan_segments->segment[i].point[num_points - 1];
		int segment_class = MASS_POINT;
		double average_distance = 0.0;
		double maximum_distance = 0.0;
		for (int j = 0; j < num_points; j++)
		{
			carmen_point_t point = virtual_scan_segments->segment[i].point[j];
			int point_type;
			// Finds the projection of the point to the line that connects v to w
			carmen_point_t point_within_line_segment = distance_from_point_to_line_segment_vw(&point_type, v, w, point);
			if (point_type == SEGMENT_TOO_SHORT)
			{
				segment_class = MASS_POINT;
				break;
			}
			else if (point_type == POINT_WITHIN_SEGMENT)
			{
				double distance = DIST2D(point, point_within_line_segment);
				if (distance > maximum_distance)
					maximum_distance = distance;
				average_distance += DIST2D(point, point_within_line_segment);
				if (j >= (num_points - 1))
				{
					average_distance = average_distance / (double) (num_points - 2);
					if (average_distance > DIST2D(v, w) / 7.0) // Revise
						segment_class = L_SHAPED;
					else
						segment_class = I_SHAPED;
				}
			}
			else
				continue;
		}

		carmen_point_t centroid = compute_segment_centroid(virtual_scan_segments->segment[i]);

		virtual_scan_segment_classes->segment[i].num_points = num_points;
		virtual_scan_segment_classes->segment[i].point = virtual_scan_segments->segment[i].point;	virtual_scan_segment_classes->segment_features[i].average_distance_from_point_to_line_segment = average_distance;
		virtual_scan_segment_classes->segment_features[i].centroid = centroid;
		virtual_scan_segment_classes->segment_features[i].first_point = v;
		virtual_scan_segment_classes->segment_features[i].last_point = w;
		virtual_scan_segment_classes->segment_features[i].maximum_distance_from_point_to_line_segment = maximum_distance;
		virtual_scan_segment_classes->segment_features[i].segment_class = segment_class;
	}

	return (virtual_scan_segment_classes);
}


//virtual_scan_box_models_t *
//fit_box_models(virtual_scan_segment_classes_t *virtual_scan_segment_classes)
//{
//// Box classes
//#define BUS			0
//#define CAR			1
//#define BIKE		2 // Motorcycles and bicycles
//#define PEDESTRIAN	3

//	virtual_scan_box_models_t *virtual_scan_box_models;
//
//	virtual_scan_box_models = (virtual_scan_box_models_t *) malloc(sizeof(virtual_scan_box_models_t));
//	virtual_scan_box_models->num_boxes = virtual_scan_segments->num_segments;
//	virtual_scan_box_models->box = (box_model_t *) malloc(sizeof(box_model_t) * virtual_scan_box_models->num_boxes);
//
//		switch (segment_class)
//		{
//			case MASS_POINT: // Represented by a point model, by which the object is parameterized by the object class and object center position
//				virtual_scan_box_models->box[i].box_class = PEDESTRIAN;
//				virtual_scan_box_models->box[i].x = centroid.x;
//				virtual_scan_box_models->box[i].y = centroid.y;
//				virtual_scan_box_models->box[i].theta = 0.0;
//				virtual_scan_box_models->box[i].width = 0.0;
//				virtual_scan_box_models->box[i].length = 0.0;
//			break;
//
//			case L_SHAPED: // Represented by a box model, by which the object is parameterized by the object class, object center position, orientation, width and length
////				virtual_scan_box_models->box[i].box_class = ;
////				virtual_scan_box_models->box[i].x = ;
////				virtual_scan_box_models->box[i].y = ;
////				virtual_scan_box_models->box[i].theta = ;
////				virtual_scan_box_models->box[i].width = ;
////				virtual_scan_box_models->box[i].length = ;
//			break;
//
//			case I_SHAPED:
////				virtual_scan_box_models->box[i].box_class = ;
////				virtual_scan_box_models->box[i].x = ;
////				virtual_scan_box_models->box[i].y = ;
////				virtual_scan_box_models->box[i].theta = ;
////				virtual_scan_box_models->box[i].width = ;
////				virtual_scan_box_models->box[i].length = ;
//			break;
//		}
//
//	return (virtual_scan_box_models);
//	}


virtual_scan_segment_classes_t *
detect_and_track_moving_objects(carmen_mapper_virtual_scan_message *virtual_scan)
{
	extended_virtual_scan_t *extended_virtual_scan = sort_virtual_scan(virtual_scan);
	virtual_scan_segments_t *virtual_scan_segments = segment_virtual_scan(extended_virtual_scan);
	virtual_scan_segment_classes_t *virtual_scan_segment_classes = classify_segments(virtual_scan_segments);
	free(virtual_scan_segments->segment);
	free(virtual_scan_segments);
//	virtual_scan_box_models_t *virtual_scan_box_models = fit_box_models(virtual_scan_segment_classes);
//	neighborhood_graph_of_hypotheses = build_or_update_neighborhood_graph_of_hypotheses(virtual_scan_box_models);
//	moving_objects = sample_moving_objects_tracks(neighborhood_graph_of_hypotheses);

	return (virtual_scan_segment_classes);
}

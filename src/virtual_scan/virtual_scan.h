/*
 * virtual_scan.h
 *
 *  Created on: Jul 17, 2017
 *      Author: claudine
 */

#ifndef SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_H_
#define SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_H_


// Segment classes
#define L_SHAPED	0
#define	I_SHAPED	1
#define MASS_POINT	2

typedef struct
{
	int num_points;
	carmen_point_t *points;
	double timestamp;
} virtual_scan_extended_t;


typedef struct
{
	int num_points;
	carmen_point_t *points;
} virtual_scan_segment_t;


typedef struct
{
	int num_segments;
	virtual_scan_segment_t *segment;
	double timestamp;
} virtual_scan_segments_t;


typedef struct
{
	carmen_point_t first_point;
	carmen_point_t last_point;
	double maximum_distance_to_line_segment;
	carmen_point_t farthest_point;
	double width;
	double length;
	int segment_class;
	carmen_point_t centroid;
	//	double average_distance_to_line_segment;
} virtual_scan_segment_features_t;


typedef struct
{
	int num_segments;
	virtual_scan_segment_t *segment;
	virtual_scan_segment_features_t *segment_features;
	double timestamp;
} virtual_scan_segment_classes_t;


typedef struct
{
	int c;
	double x;
	double y;
	double theta;
	double width;
	double length;
} virtual_scan_box_model_t;


typedef struct
{
	int num_boxes;
	virtual_scan_box_model_t *box;
} virtual_scan_box_models_t;


typedef struct
{
	int num_box_model_hypotheses;
	virtual_scan_box_models_t *box_model_hypotheses;
	int last_box_model_hypotheses;
	double timestamp;
} virtual_scan_box_model_hypotheses_t;


typedef struct
{
	int category;
	double width;
	double length;
} virtual_scan_category_t;

virtual_scan_extended_t *
sort_virtual_scan(carmen_mapper_virtual_scan_message *virtual_scan);

void
virtual_scan_free_extended(virtual_scan_extended_t *virtual_scan_extended);

virtual_scan_box_models_t *
virtual_scan_new_box_models(void);

virtual_scan_box_model_t *
virtual_scan_append_box(virtual_scan_box_models_t *models);

virtual_scan_box_model_hypotheses_t *
virtual_scan_new_box_model_hypotheses(int length);

virtual_scan_box_models_t *
virtual_scan_get_box_models(virtual_scan_box_model_hypotheses_t *hypotheses, int i);

virtual_scan_segment_classes_t *
virtual_scan_extract_segments(virtual_scan_extended_t *virtual_scan_extended);

void
virtual_scan_free_segment_classes(virtual_scan_segment_classes_t *virtual_scan_segment_classes);

virtual_scan_box_model_hypotheses_t *
virtual_scan_fit_box_models(virtual_scan_segment_classes_t *virtual_scan_segment_classes);

void
virtual_scan_free_box_model_hypotheses(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses);

void
virtual_scan_publish_box_models(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses);

#endif /* SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_H_ */

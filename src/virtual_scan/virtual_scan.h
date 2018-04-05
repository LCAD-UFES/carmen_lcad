/*
 * virtual_scan.h
 *
 *  Created on: Jul 17, 2017
 *	  Author: claudine
 */

#ifndef SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_H_
#define SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_H_


// Segment classes
#define L_SHAPED	1
#define	I_SHAPED	2
#define MASS_POINT	3

#define	BUS			'B' // Width: 2,4 m to 2,6 m; Length: 10 m to 14 m;
#define	CAR			'C' // Width: 1,8 m to 2,1; Length: 3,9 m to 5,3 m
#define	BIKE		'b' // Width: 1,20 m; Length: 2,20 m
#define	PEDESTRIAN	'P'

#define CHILD_EDGE 		'C'
#define PARENT_EDGE 	'P'
#define SIBLING_EDGE 	'S'

#define PROB_THRESHOLD	-2.14

#define	POINT_WITHIN_SEGMENT		0
#define	SEGMENT_TOO_SHORT			1
#define	POINT_BEFORE_SEGMENT		2
#define	POINT_AFTER_SEGMENT			3

#define PEDESTRAIN_SIZE	0.5 // Pedestrian approximate size (from the top) in meters
#define	L_SMALL_SEGMENT_AS_A_PROPORTION_OF_THE_LARGE	0.3

#define	MCMC_MAX_ITERATIONS	300

#define GAMMA	0.75
#define VMAX	(120.0 / 3.6)

#define NUMBER_OF_FRAMES_T 10


typedef struct
{
	int num_points;
	carmen_point_t *points;
	carmen_point_t globalpos;
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
	virtual_scan_segment_t *box_points;
} virtual_scan_box_models_t;


typedef struct
{
	int num_box_model_hypotheses;
	virtual_scan_box_models_t *box_model_hypotheses; // Varios boxes por segmento do scan
	int last_box_model_hypotheses;
	double timestamp;
} virtual_scan_box_model_hypotheses_t;


typedef struct
{
	int category;
	double width;
	double length;
} virtual_scan_category_t;


typedef struct
{
	int size;
	int *edge;
	int *edge_type;
} virtual_scan_box_model_hypothesis_edges_t;


typedef struct
{
	int index;
	virtual_scan_box_model_t hypothesis;
	virtual_scan_segment_t hypothesis_points;
	int zi;

	double dn;
	double c2;
	double c3;

	double timestamp;
} virtual_scan_box_model_hypothesis_t;


typedef struct
{
	int size;
	virtual_scan_box_model_hypothesis_t **box_model_hypothesis;
	virtual_scan_box_model_hypothesis_edges_t **box_model_hypothesis_edges;
	bool *vertex_selected;

	double *last_frames_timetamps;
	int number_of_frames_filled;
} virtual_scan_neighborhood_graph_t;


typedef struct
{
	int size;
	virtual_scan_box_model_hypothesis_t *box_model_hypothesis;
} virtual_scan_track_t;


typedef struct
{
	int size;
	virtual_scan_track_t **tracks;
} virtual_scan_track_set_t;


typedef struct
{

} virtual_scan_moving_objects_t;


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
virtual_scan_free_scan_extended(virtual_scan_extended_t *virtual_scan_extended);

void
virtual_scan_free_segment_classes(virtual_scan_segment_classes_t *virtual_scan_segment_classes);

virtual_scan_box_model_hypotheses_t *
virtual_scan_fit_box_models(virtual_scan_segment_classes_t *virtual_scan_segment_classes);

void
virtual_scan_free_box_model_hypotheses(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses);

void
virtual_scan_free_moving_objects(virtual_scan_moving_objects_t *moving_objects);

void
virtual_scan_publish_box_models(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses);

int
virtual_scan_num_box_models(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses);

virtual_scan_neighborhood_graph_t *
virtual_scan_update_neighborhood_graph(virtual_scan_neighborhood_graph_t *neighborhood_graph, virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses);

virtual_scan_moving_objects_t *
virtual_scan_infer_moving_objects(virtual_scan_neighborhood_graph_t *neighborhood_graph);

#endif /* SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_H_ */

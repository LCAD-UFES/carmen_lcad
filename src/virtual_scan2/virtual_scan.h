/*
 * virtual_scan.h
 *
 *  Created on: Jul 17, 2017
 *	  Author: claudine
 */

#ifndef SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_H_
#define SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_H_


// Segment classes
#define L_SHAPED	0
#define	I_SHAPED	1
#define MASS_POINT	2

#define GET_MIN_V_PER_OBJECT_CLASS(c) ((c == BUS)? 0.5: (c == CAR)? 0.5: (c == BIKE)? 0.5: (c == PEDESTRIAN)? 0.2: 0.5)

#define CHILD_EDGE 		'C'
#define PARENT_EDGE 	'P'
#define SIBLING_EDGE 	'S'

#define PROB_THRESHOLD	-2.14

#define	POINT_WITHIN_SEGMENT		0
#define	SEGMENT_TOO_SHORT			1
#define	POINT_BEFORE_SEGMENT		2
#define	POINT_AFTER_SEGMENT			3

#define DISTANCE_BETWEEN_SEGMENTS	1.0
#define	L_SMALL_SEGMENT_AS_A_PROPORTION_OF_THE_LARGE	0.3
#define PEDESTRIAN_RADIUS			0.5 	// pedestrian approximate radius (from the top) in meters
#define MINIMUN_CLUSTER_SIZE		1		// in points
#define	MIN_SEGMENT_SIZE			5
#define	MCMC_MAX_ITERATIONS	300

//#define GAMMA	0.75
#define GAMMA	0.15
#define VMAX	(120.0 / 3.6)

#define NUMBER_OF_FRAMES_T 20

//#define MAX_VELODYNE_SEGMENT_DISTANCE 15.0


typedef struct
{
	int zi;
	int sensor;
	int sensor_id;
	carmen_point_t sensor_pos;
	carmen_point_t global_pos;
	double sensor_v;
	double sensor_w;
	carmen_point_t centroid;
	double precise_timestamp;

	int num_points;
	carmen_point_t *points;
} virtual_scan_segment_t;


typedef struct
{
	int num_segments;
	virtual_scan_segment_t *segment;
} virtual_scan_segments_t;


typedef struct
{
	carmen_point_t first_point;
	carmen_point_t last_point;
	carmen_point_t farthest_point;
	carmen_point_t main_line_first_point;
	carmen_point_t main_line_last_point;
	double width;
	double length;
	int segment_class;
} virtual_scan_segment_features_t;


typedef struct
{
	int num_segments;
	virtual_scan_segment_t *segment;
	virtual_scan_segment_features_t *segment_features;
} virtual_scan_segment_classes_t;


typedef struct
{
	int c;
	double x;
	double y;
	double theta;
	double width;
	double length;

	// O Velodyne roda no sentido horario
	//
	// Numeracao dos cantos de um objeto movel
	// 1--0
	//  ||
	// 2--3
	//

	// Uma extremidade eh a primeira do segmento (angulo) e a outra eh a ultima (I)
	// 	IARA
	//
	//  1--0
	//

	// Uma extremidade eh a ultima do segmento (angulo) e a outra eh a intermediaria (L)
	//    IARA
	//
	//  1--0
	//    |
	//     3

	// Uma extremidade eh a primeira do segmento e a outra eh a ultima (I)
	//   0
	// 	|   IARA
	//   3

	// Uma extremidade eh a ultima do segmento e a outra eh a intermediaria (L)
	//     0
	//    |
	//   --
	//  2  3
	//
	// 	  IARA
	//

	// Uma extremidade eh a primeira do segmento e a outra eh a ultima (I)
	//
	//
	//   2--3
	//
	// 	 IARA
	//

	// Uma extremidade eh a primeira do segmento e a outra eh a intermediaria (L)
	//    1
	//     |
	//	  2--3
	//
	// 	IARA
	//

	// Uma extremidade eh a primeira do segmento e a outra eh a ultima (I)
	//
	//       1
	// 	IARA  |
	//       2

	// Uma extremidade eh a ultima do segmento (angulo) e a outra eh a intermediaria (L)
	// 	IARA
	//
	//    1--0
	//     |
	//    2
	//

	carmen_point_t point0;
	bool point0_valid;
	carmen_point_t point1;
	bool point1_valid;
	carmen_point_t point2;
	bool point2_valid;
	carmen_point_t point3;
	bool point3_valid;
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
	double frame_timestamp;
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
	double x;
	double y;
	double theta;
	double v;
	double a;
	double w;
} virtual_scan_hypothesis_state_t;


typedef struct
{
	int index;
	virtual_scan_box_model_t hypothesis;
	virtual_scan_segment_t hypothesis_points;
	virtual_scan_hypothesis_state_t hypothesis_state;

	double dn;
	double c2;
	double c3;

	bool already_examined;
	double frame_timestamp;
} virtual_scan_box_model_hypothesis_t;


typedef struct
{
	int size;
	virtual_scan_box_model_hypothesis_t *box_model_hypothesis;
	int track_id;
} virtual_scan_track_t;


typedef struct
{
	int size;
	virtual_scan_track_t **tracks;
	bool *vertex_selected;
} virtual_scan_track_set_t;


carmen_mapper_virtual_scan_message *
sort_virtual_scan(carmen_mapper_virtual_scan_message *virtual_scan);

carmen_mapper_virtual_scan_message *
filter_virtual_scan(carmen_mapper_virtual_scan_message *virtual_scan_extended);

virtual_scan_box_models_t *
virtual_scan_new_box_models(void);

virtual_scan_box_model_t *
virtual_scan_append_box(virtual_scan_box_models_t *models);

virtual_scan_box_model_hypotheses_t *
virtual_scan_new_box_model_hypotheses(int length);

virtual_scan_box_models_t *
virtual_scan_get_box_models(virtual_scan_box_model_hypotheses_t *hypotheses, int i);

virtual_scan_segment_classes_t *
virtual_scan_extract_segments(carmen_mapper_virtual_scan_message *virtual_scan_extended);

carmen_mapper_virtual_scan_message *
copy_virtual_scan_message(carmen_mapper_virtual_scan_message *virtual_scan);

void
virtual_scan_free_scan_extended(carmen_mapper_virtual_scan_message *virtual_scan_extended);

void
virtual_scan_free_segment_classes(virtual_scan_segment_classes_t *virtual_scan_segment_classes);

virtual_scan_box_model_hypotheses_t *
virtual_scan_fit_box_models(virtual_scan_segment_classes_t *virtual_scan_segment_classes, double frame_timestamp);

void
virtual_scan_free_box_model_hypotheses(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses);

void
virtual_scan_free_moving_objects(carmen_moving_objects_point_clouds_message *moving_objects);

void
virtual_scan_publish_box_models(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses);

int
virtual_scan_num_box_models(virtual_scan_box_model_hypotheses_t *virtual_scan_box_model_hypotheses);

void
virtual_scan_tracker_initialize(void);

void
virtual_scan_tracker_finalize(void);

virtual_scan_track_set_t *
virtual_scan_infer_moving_objects(carmen_mapper_virtual_scan_message *virtual_scan_extended, virtual_scan_segment_classes_t *virtual_scan_segment_classes, double frame_timestamp);

virtual_scan_track_set_t *
virtual_scan_infer_moving_objects2(carmen_mapper_virtual_scan_message *virtual_scan_extended, double frame_timestamp);

void 
virtual_scan_show_tracks();

void 
virtual_scan_show_plots(int sensor_id);

double
probability_of_track_set_given_measurements(virtual_scan_track_set_t *track_set, bool print = false);

void
update_hypotheses_state(virtual_scan_track_t *track);

carmen_point_t
distance_from_point_to_line_segment_vw(int *point_in_trajectory_is, carmen_point_t v, carmen_point_t w, carmen_point_t p);

double
distance_from_point_to_line_segment_vw(carmen_position_t v, carmen_position_t w, carmen_point_t p);

#endif /* SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_H_ */

/*
 * virtual_scan.h
 *
 *  Created on: Jul 17, 2017
 *      Author: claudine
 */

#ifndef SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_H_
#define SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_H_


typedef struct
{
	int num_points;
	carmen_point_t *point;
} extended_virtual_scan_t;


typedef struct
{
	int num_points;
	carmen_point_t *point;
} virtual_scan_segment_t;


typedef struct
{
	int box_class;
	double x;
	double y;
	double theta;
	double width;
	double length;
} box_model_t;


typedef struct
{
	int num_boxes;
	box_model_t *box;
} virtual_scan_box_model_t;


void
detect_and_track_moving_objects(carmen_mapper_virtual_scan_message *virtual_scan);


#endif /* SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_H_ */

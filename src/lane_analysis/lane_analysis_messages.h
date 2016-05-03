/*********************************************************
 Lane Analysis Module
 *********************************************************/

#ifndef CARMEN_LANE_ANALYSIS_MESSAGES_H
#define CARMEN_LANE_ANALYSIS_MESSAGES_H

#include "global.h"
#include <vector>

#ifdef __cplusplus
extern "C"
{
#endif

/* FULL MESSAGE CONTENT
 * - DONE: struct { std::vector<carmen_vector_2D_t> left, right; } lane_position;
 * - DONE: struct { carmen_vector_2D_t point_bottom, point_top, direction; double width; } lane_base;
 * - DONE: struct { int left, right; } lmt;
 * - DONE: struct { int left, right; } adjacent_lanes;
 * int lane_change; // TODO: change for int, where -1 = to left, 1 = to right and 0 = none
 * double lane_deviation;
 * - DONE: int trustworthy_height;
 * int isKalmanNull;
 * double execution_time;
 * double car_position_x;
 * - DONE: double timestamp; 		// obrigatory!
 * - DONE: char *host; 				// obrigatory!
 */

// ==============================================================
typedef struct {
	int num_outputs_left, num_outputs_right;
	carmen_vector_2D_t *left, *right;
	carmen_vector_2D_t point_bottom, point_top, direction;
	double lane_width;
	int trustworthy_height; // TODO: change to distance in meters after calibration
	double timestamp; // obrigatory!
	char *host; // obrigatory!
} carmen_elas_lane_estimation_message;

#define CARMEN_ELAS_LANE_ESTIMATION_NAME "carmen_elas_lane_estimation_message"
#define CARMEN_ELAS_LANE_ESTIMATION_FMT "{int,int,<vector_2D:1>,<vector_2D:2>,vector_2D,vector_2D,vector_2D,double,int,double,string}"

// ==============================================================
typedef struct {
	int left, right;
	double timestamp;
	char *host;
} carmen_elas_lane_markings_type_message;

#define CARMEN_ELAS_LANE_MARKINGS_TYPE_NAME "carmen_elas_lane_markings_type_message"
#define CARMEN_ELAS_LANE_MARKINGS_TYPE_FMT "{int,int,double,string}"

// ==============================================================
typedef struct {
	int left, right;
	double timestamp;
	char *host;
} carmen_elas_adjacent_lanes_message;

#define CARMEN_ELAS_ADJACENT_LANES_NAME "carmen_elas_adjacent_lanes_message"
#define CARMEN_ELAS_ADJACENT_LANES_FMT "{int,int,double,string}"


#ifdef __cplusplus
}
#endif

#endif

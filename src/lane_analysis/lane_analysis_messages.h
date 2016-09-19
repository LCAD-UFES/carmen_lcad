/*********************************************************
 Lane Analysis Module
 *********************************************************/

#ifndef CARMEN_LANE_ANALYSIS_MESSAGES_H
#define CARMEN_LANE_ANALYSIS_MESSAGES_H

#include "global.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ==============================================================
/* OLD MESSAGE
typedef struct {
	int num_outputs_left, num_outputs_right;
	carmen_vector_2D_t *left, *right;
	carmen_vector_2D_t *left_ipm, *right_ipm;
	carmen_vector_2D_t point_bottom, point_top, direction;
	double lane_deviation;
	double lane_width;
	int trustworthy_height; // TODO: change to distance in meters after calibration
	double scale_x, scale_y;
	double timestamp; // obrigatory!
	char *host; // obrigatory!
} carmen_elas_lane_estimation_message;
*/

typedef struct {
	int num_control_points;
	carmen_vector_3D_t *left_control_points, *right_control_points;
	int left_lmt, right_lmt;
	double timestamp; // obrigatory!
	char *host; // obrigatory!
} carmen_elas_lane_analysis_message;

#define CARMEN_ELAS_LANE_ANALYSIS_NAME "carmen_elas_lane_analysis_message"
#define CARMEN_ELAS_LANE_ANALYSIS_FMT "{int,<vector_3D:1>,<vector_3D:1>,int,int,double,string}"

#ifdef __cplusplus
}
#endif

#endif

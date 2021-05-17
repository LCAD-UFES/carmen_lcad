#ifndef CARMEN_NAVIGATOR_SPLINE_MESSAGES_H
#define CARMEN_NAVIGATOR_SPLINE_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	carmen_robot_and_trailer_traj_point_t *goal_list;
	int size;
	double timestamp;
	char *host;
} carmen_navigator_spline_path_message;

#define		CARMEN_NAVIGATOR_SPLINE_PATH_NAME		"carmen_navigator_spline_path"
#define		CARMEN_NAVIGATOR_SPLINE_PATH_FMT		"{<{double, double, double, double, double, double}:2>,int,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}

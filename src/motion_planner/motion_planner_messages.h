/*
 * motion_planner_messages.h
 *
 *  Created on: 20/09/2012
 *      Author: romulo
 */

#ifndef MOTION_PLANNER_MESSAGES_H_
#define MOTION_PLANNER_MESSAGES_H_

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
	carmen_robot_and_trailers_traj_point_t *path;
	int path_size;
	int algorithm; //see carmen_navigator_ackerman_algorithm_t
	double timestamp;
	char *host;
} carmen_motion_planner_path_message;

#define CARMEN_MOTION_PLANNER_PATH_NAME "carmen_motion_planner_path"
#define CARMEN_MOTION_PLANNER_PATH_FMT "{<{double, double, double, int, [double:5], double, double}:2>, int, int, double, string}"


#ifdef __cplusplus
}
#endif


#endif /* MOTION_PLANNER_MESSAGES_H_ */

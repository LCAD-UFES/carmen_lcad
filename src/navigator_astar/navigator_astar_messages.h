#ifndef CARMEN_NAVIGATOR_ASTAR_MESSAGES_H
#define CARMEN_NAVIGATOR_ASTAR_MESSAGES_H

#include <carmen/global.h>
#include <carmen/map.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	carmen_robot_and_trailers_traj_point_t *points;
	int length;
	int capacity;
} carmen_planner_path_t, *carmen_planner_path_p;

typedef struct {
	carmen_robot_and_trailers_traj_point_t *goal_list;
	int size;
	double timestamp;
	char *host;
} carmen_navigator_ackerman_astar_goal_list_message;

typedef struct {
	carmen_robot_and_trailers_traj_point_t robot;
	carmen_robot_and_trailers_traj_point_t goal;
	carmen_planner_path_t path;
	int goal_set;
} carmen_planner_status_t, *carmen_planner_status_p;


#define		CARMEN_ASTAR_GOAL_LIST_NAME		"carmen_navigator_ackerman_astar_goal_list"
#define		CARMEN_ASTAR_GOAL_LIST_FMT		"{<{double, double, double, double, double, double}:2>,int,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}

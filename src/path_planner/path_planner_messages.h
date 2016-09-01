#ifndef CARMEN_NAVIGATOR_ASTAR_MESSAGES_H
#define CARMEN_NAVIGATOR_ASTAR_MESSAGES_H

#include <carmen/global.h>
#include <carmen/map.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	carmen_ackerman_traj_point_p points;
	int length;
	int capacity;
} carmen_planner_path_t, *carmen_planner_path_p;

typedef struct {
	carmen_ackerman_traj_point_t *goal_list;
	int size;
	double timestamp;
	char *host;
} carmen_navigator_ackerman_astar_goal_list_message;

typedef struct {
	carmen_ackerman_traj_point_t robot;
	carmen_ackerman_traj_point_t goal;
	carmen_planner_path_t path;
	int goal_set;
} carmen_planner_status_t, *carmen_planner_status_p;



#define		CARMEN_ASTAR_GOAL_LIST_NAME		"carmen_navigator_ackerman_astar_goal_list"
#define		CARMEN_ASTAR_GOAL_LIST_FMT		"{<{double, double, double, double, double}:2>,int,double,string}"


typedef struct
{
	int number_of_poses;
	int number_of_poses_back;
	carmen_ackerman_traj_point_t *poses;
	carmen_ackerman_traj_point_t *poses_back;
	// int *signals_annotations;
	int *annotations;
	double timestamp;
	char *host;
} carmen_path_planner_road_profile_message;

#define CARMEN_PATH_PLANNER_ROAD_PROFILE_MESSAGE_NAME "carmen_path_planner_road_profile_message"
#define CARMEN_PATH_PLANNER_ROAD_PROFILE_MESSAGE_FMT "{int, int, <{double, double, double, double, double}:1>, <{double, double, double, double, double}:2>, <int:1>, double, string}"


#ifdef __cplusplus
}
#endif

#endif

// @}

#ifndef RRT_PLANNER_MESSAGE_H
#define RRT_PLANNER_MESSAGE_H

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	carmen_point_t p1, p2;
} carmen_rrt_edge;

typedef struct
{
	carmen_rrt_edge *edges;
	int	   num_edges;
	double timestamp;
	char  *host;
} carmen_rrt_planner_tree_message;

#define CARMEN_RRT_PLANNER_ROBOT_TREE_NAME "carmen_rrt_planner_robot_tree"
#define CARMEN_RRT_PLANNER_ROBOT_TREE_FMT "{<{{double, double, double}, {double, double, double}}:2>, int, double, string}"

#define CARMEN_RRT_PLANNER_GOAL_TREE_NAME "carmen_rrt_planner_goal_tree"
#define CARMEN_RRT_PLANNER_GOAL_TREE_FMT CARMEN_RRT_PLANNER_ROBOT_TREE_FMT

typedef struct
{
	int goal_set;                  /**< Is there a current goal? */
	carmen_point_t goal;           /**< Undefined if goal_set is 0 */
	double timestamp;
	char  *host;
} carmen_rrt_planner_status_message;

#define CARMEN_RRT_PLANNER_STATUS_NAME       "carmen_rrt_planner_status"
#define CARMEN_RRT_PLANNER_STATUS_FMT        "{int,{double, double, double},double,string}"

typedef struct
{
	double x, y, theta;
	double timestamp;
	char  *host;
} carmen_rrt_planner_set_goal_message;

#define CARMEN_RRT_PLANNER_SET_GOAL_NAME "carmen_rrt_planner_set_goal"
#define CARMEN_RRT_PLANNER_SET_GOAL_FMT "{double,double,double,double,string}"

typedef struct
{
	carmen_robot_and_trailers_traj_point_t *path;
	int	   path_length;
	double timestamp;
	char  *host;
} carmen_rrt_planner_plan_message;

#define CARMEN_RRT_PLANNER_PLAN_NAME "carmen_rrt_planner_plan"
#define CARMEN_RRT_PLANNER_PLAN_FMT "{<{double, double, double, int, [double:5], double, double}:2>,int,double,string}"

typedef carmen_default_message carmen_rrt_planner_go_message;
#define CARMEN_RRT_PLANNER_GO "carmen_rrt_planner_go"

typedef carmen_default_message carmen_rrt_planner_stop_message;
#define CARMEN_RRT_PLANNER_STOP "carmen_rrt_planner_stop"

#ifdef __cplusplus
}
#endif

#endif

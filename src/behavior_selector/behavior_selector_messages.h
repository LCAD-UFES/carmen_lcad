/*
 * behavior_selector_messages.h
 *
 *  Created on: 28/09/2012
 *      Author: romulo, Alberto
 */

#ifndef BEHAVIOR_SELECTOR_MESSAGES_H_
#define BEHAVIOR_SELECTOR_MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>

typedef enum
{
	BEHAVIOR_SELECTOR_FOLLOW_ROUTE,
	BEHAVIOR_SELECTOR_PARK,
	BEHAVIOR_SELECTOR_HUMAN_INTERVENTION
} carmen_behavior_selector_task_t;


typedef enum
{
	FREE_RUN_GOAL1,
	MOVING_OBSTACLE_GOAL1, MOVING_OBSTACLE_GOAL2, MOVING_OBSTACLE_GOAL3,
	OBSTACLE_GOAL,
	DYNAMIC_ANNOTATION_GOAL,
	ANNOTATION_GOAL1, ANNOTATION_GOAL2, ANNOTATION_GOAL3, ANNOTATION_GOAL_STOP,
	FREE_RUN_GOAL2,
	SWITCH_VELOCITY_SIGNAL_GOAL,
	FINAL_GOAL
} carmen_behavior_selector_goal_type_t;


typedef enum
{
	Initializing,
	Stopped,
	Free_Running,
	Free_Reverse_Running,
	Stopping_Behind_Moving_Object,
	Stopped_Behind_Moving_Object_S0,
	Stopped_Behind_Moving_Object_S1,
	Stopped_Behind_Moving_Object_S2,
	Stopping_At_Red_Traffic_Light,
	Stopped_At_Red_Traffic_Light_S0,
	Stopped_At_Red_Traffic_Light_S1,
	Stopped_At_Red_Traffic_Light_S2,
	Stopping_At_Busy_Pedestrian_Track,
	Stopped_At_Busy_Pedestrian_Track_S0,
	Stopped_At_Busy_Pedestrian_Track_S1,
	Stopped_At_Busy_Pedestrian_Track_S2,
	Stopping_At_Yield,
	Stopped_At_Yield_S0,
	Stopped_At_Yield_S1,
	Stopped_At_Yield_S2,
	Stopping_At_Stop_Sign,
	Stopped_At_Stop_Sign_S0,
	Stopped_At_Stop_Sign_S1,
	Stopped_At_Stop_Sign_S2,
	Stopping_To_Reverse,
	Stopped_At_Reverse_S0,
	Stopped_At_Reverse_S1,
	Stopped_At_Reverse_S2,
	Stopping_To_Go_Forward,
	Stopped_At_Go_Forward_S0,
	Stopped_At_Go_Forward_S1,
	Stopped_At_Go_Forward_S2,
	End_Of_Path_Reached,
	End_Of_Path_Reached2,
	Recovering_From_Error
} carmen_behavior_selector_low_level_state_t;

typedef enum
{
	CARMEN_BEHAVIOR_SELECTOR_FRENET,
	CARMEN_BEHAVIOR_SELECTOR_A_STAR,
	CARMEN_BEHAVIOR_SELECTOR_RRT,
	CARMEN_BEHAVIOR_SELECTOR_RDDF,
	CARMEN_BEHAVIOR_SELECTOR_GRADIENT,
	CARMEN_BEHAVIOR_SELECTOR_INVALID_PLANNER
} carmen_behavior_selector_algorithm_t;

typedef struct
{
	carmen_behavior_selector_algorithm_t algorithm;  // algoritmo que será usado na missao
	carmen_behavior_selector_task_t task;
	double timestamp;
	char *host;
} carmen_behavior_selector_set_algorithm_message;

#define		CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME		"carmen_behavior_selector_set_algorithm"
#define		CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_FMT		"{int, int, double, string}"

typedef struct
{
	carmen_behavior_selector_task_t task;
	double timestamp;
	char *host;
} carmen_behavior_selector_set_task_message;

#define		CARMEN_BEHAVIOR_SELECTOR_SET_TASK_NAME		"carmen_behavior_selector_set_task"
#define		CARMEN_BEHAVIOR_SELECTOR_SET_TASK_FMT		"{int, double, string}"

typedef struct
{
	carmen_behavior_selector_task_t task;
	carmen_behavior_selector_algorithm_t algorithm;

	carmen_behavior_selector_low_level_state_t low_level_state;

	double timestamp;
	char *host;
} carmen_behavior_selector_state_message;

#define		CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME		"carmen_behavior_selector_current_state_name"
#define		CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_FMT		"{int, int, int, double, string}"

typedef struct
{
	carmen_point_t goal;
	double timestamp;
	char *host;
} carmen_behavior_selector_add_goal_message;

#define		CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_NAME		"carmen_behavior_selector_add_goal_name"
#define		CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_FMT		"{{double, double, double}, double, string}"

typedef carmen_default_message carmen_behavior_selector_clear_goal_list_message;

#define		CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_NAME 	"carmen_behavior_selector_clear_goal_list"
#define 	CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_FMT	CARMEN_DEFAULT_MESSAGE_FMT

typedef carmen_default_message carmen_behavior_selector_remove_goal_message;

#define		CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_NAME 	"carmen_behavior_selector_remove_goal"
#define 	CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_FMT	CARMEN_DEFAULT_MESSAGE_FMT

typedef struct
{
    int number_of_poses;
    int number_of_poses_back;
    carmen_robot_and_trailer_traj_point_t *poses;
    carmen_robot_and_trailer_traj_point_t *poses_back;
    int *annotations;
    int *annotations_codes;
	int goal_list_size;
	carmen_robot_and_trailer_traj_point_t *goal_list;
    double timestamp;
    char *host;
} carmen_behavior_selector_path_goals_and_annotations_message;

#define CARMEN_BEHAVIOR_SELECTOR_PATH_GOALS_AND_ANNOTATIONS_MESSAGE_NAME "carmen_behavior_selector_path_goals_and_annotations_message"
#define CARMEN_BEHAVIOR_SELECTOR_PATH_GOALS_AND_ANNOTATIONS_MESSAGE_FMT "{int, int, <{double, double, double, double, double, double}:1>, <{double, double, double, double, double, double}:2>, <int:1>, <int:1>, int, <{double, double, double, double, double, double}:7>, double, string}"

#ifdef __cplusplus
}
#endif

#endif /* BEHAVIOR_SELECTOR_MESSAGES_H_ */

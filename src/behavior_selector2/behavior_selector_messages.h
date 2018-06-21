/*
 * behavior_selector_messages.h
 *
 *  Created on: 28/09/2012
 *      Author: romulo
 */

#ifndef BEHAVIOR_SELECTOR_MESSAGES_H_
#define BEHAVIOR_SELECTOR_MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>

typedef enum
{
	BEHAVIOR_SELECTOR_FOLLOWING_LANE, BEHAVIOR_SELECTOR_PARKING, BEHAVIOR_SELECTOR_HUMAN_INTERVENTION
} carmen_behavior_selector_state_t;


typedef enum
{
	FREE_RUN_GOAL1,
	MOVING_OBSTACLE_GOAL1, MOVING_OBSTACLE_GOAL2,
	OBSTACLE_GOAL,
	DYNAMIC_ANNOTATION_GOAL,
	ANNOTATION_GOAL1, ANNOTATION_GOAL2, ANNOTATION_GOAL3,
	FREE_RUN_GOAL2,
} carmen_behavior_selector_goal_type_t;


typedef enum
{
	Initializing,
	Stopped,
	Free_Running,
	Following_Moving_Object,
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
	Stopping_At_Stop_Sign,
	Stopped_At_Stop_Sign_S0,
	Stopped_At_Stop_Sign_S1
} carmen_behavior_selector_low_level_state_t;

typedef enum
{
	none = 0,
	stopped,
	free_running,
	following_moving_object,
	stopping_behind_moving_object,
	stopped_behind_moving_object,
	stopping_at_red_light,
	stopped_at_red_light,
	stopping_at_stop_sign,
	stopped_at_stop_sign,
} carmen_behavior_selector_operation_mode_t;


typedef struct {
	carmen_ackerman_traj_point_t *goal_list;
	int size;
	double timestamp;
	char *host;
} carmen_behavior_selector_goal_list_message;

#define		CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME		"carmen_behavior_selector_goal_list"
#define		CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_FMT		"{<{double, double, double, double, double}:2>,int,double,string}"

typedef struct {
	carmen_ackerman_traj_point_t *goal_list;
	int size;
	double timestamp;
	char *host;
} carmen_behavior_selector_goal_list_rddf_message;

#define		CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_NAME		"carmen_behavior_selector_goal_list_rddf"
#define		CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_FMT		"{<{double, double, double, double, double}:2>,int,double,string}"

typedef enum {
	CARMEN_BEHAVIOR_SELECTOR_GRADIENT, CARMEN_BEHAVIOR_SELECTOR_A_STAR, CARMEN_BEHAVIOR_SELECTOR_RRT, CARMEN_BEHAVIOR_SELECTOR_RDDF, CARMEN_BEHAVIOR_SELECTOR_INVALID_PLANNER
} carmen_behavior_selector_algorithm_t;

typedef enum {
	CARMEN_BEHAVIOR_SELECTOR_USER_GOAL, CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL, CARMEN_BEHAVIOR_SELECTOR_PATH_PLANNER_GOAL
} carmen_behavior_selector_goal_source_t;

typedef struct {
	carmen_behavior_selector_algorithm_t algorithm;
	carmen_behavior_selector_state_t state; //algoritmo que será aplicado ao state
	double timestamp;
	char *host;
} carmen_behavior_selector_set_algorithm_message;

#define		CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME		"carmen_behavior_selector_set_algorithm"
#define		CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_FMT		"{int, int, double, string}"

typedef struct {
	carmen_behavior_selector_state_t state;
	double timestamp;
	char *host;
} carmen_behavior_selector_set_state_message;

#define		CARMEN_BEHAVIOR_SELECTOR_SET_STATE_NAME		"carmen_behavior_selector_set_state"
#define		CARMEN_BEHAVIOR_SELECTOR_SET_STATE_FMT		"{int, double, string}"

typedef struct {
	carmen_behavior_selector_goal_source_t goal_source;
	double timestamp;
	char *host;
} carmen_behavior_selector_set_goal_source_message;

#define		CARMEN_BEHAVIOR_SELECTOR_SET_GOAL_SOURCE_NAME	"carmen_behavior_selector_set_goal_source"
#define		CARMEN_BEHAVIOR_SELECTOR_SET_GOAL_SOURCE_FMT	"{int, double, string}"


typedef struct {
	carmen_behavior_selector_algorithm_t algorithm;
	carmen_behavior_selector_state_t state;

	carmen_behavior_selector_algorithm_t following_lane_algorithm;
	carmen_behavior_selector_algorithm_t parking_algorithm;

	carmen_behavior_selector_goal_source_t goal_source;

	carmen_behavior_selector_low_level_state_t low_level_state;
	carmen_behavior_selector_operation_mode_t behaviour_seletor_mode;


	double timestamp;
	char *host;
} carmen_behavior_selector_state_message;

#define		CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME		"carmen_behavior_selector_current_state_name"
#define		CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_FMT		"{int, int, int, int, int, int, int, double, string}"

typedef struct {
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
    carmen_ackerman_traj_point_t *poses;
    carmen_ackerman_traj_point_t *poses_back;
    // int *signals_annotations;
    int *annotations;
    double timestamp;
    char *host;
} carmen_behavior_selector_road_profile_message;

#define CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME "carmen_behavior_selector_road_profile_message"
#define CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_FMT "{int, int, <{double, double, double, double, double}:1>, <{double, double, double, double, double}:2>, <int:1>, double, string}"

#ifdef __cplusplus
}
#endif

#endif /* BEHAVIOR_SELECTOR_MESSAGES_H_ */

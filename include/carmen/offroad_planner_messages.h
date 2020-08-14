/*
 * offroad_planner_messages.h
 *
 *  Created on: 14/05/2020
 *      Author: Alberto
 */

#ifndef OFFROAD_PLANNER_MESSAGES_H_
#define OFFROAD_PLANNER_MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>


typedef enum OFFROAD_PLANNER_FEEDBACK
{
	PLAN_OK,
	ALREADY_IN_GOAL,
	GOAL_OVER_OBSTACLE,
	START_OVER_OBSTACLE,
	TIMEOUT,
	GOAL_NOT_FOUND,
	OBSTACLE_DISTANCE_MAP_NOT_AVAILABLE,
	COULD_NOT_COMPUTE_GOAL_DISTANCE_MAP
} carmen_offroad_planner_feedback_t;

#define print_offroad_planner_feedback(x) ( \
	(x == PLAN_OK)? "PLAN_OK": \
	(x == ALREADY_IN_GOAL)? "ALREADY_IN_GOAL": \
	(x == GOAL_OVER_OBSTACLE)? "GOAL_OVER_OBSTACLE": \
	(x == START_OVER_OBSTACLE)? "START_OVER_OBSTACLE": \
	(x == TIMEOUT)? "TIMEOUT": \
	(x == GOAL_NOT_FOUND)? "GOAL_NOT_FOUND": \
	(x == OBSTACLE_DISTANCE_MAP_NOT_AVAILABLE)? "OBSTACLE_DISTANCE_MAP_NOT_AVAILABLE": "COULD_NOT_COMPUTE_GOAL_DISTANCE_MAP")


typedef struct
{
	carmen_offroad_planner_feedback_t offroad_planner_feedback;
	int number_of_poses;
	carmen_ackerman_traj_point_t *poses;
	int pose_id;
	carmen_ackerman_traj_point_t transition_pose;

    double timestamp;
    char *host;
} carmen_offroad_planner_plan_message;

#define		CARMEN_OFFROAD_PLANNER_PLAN_MESSAGE_NAME	"carmen_offroad_planner_plan_message"
#define		CARMEN_OFFROAD_PLANNER_PLAN_MESSAGE_FMT		"{int, int, <{double, double, double, double, double}:2>, int, {double, double, double, double, double}, double, string}"

#ifdef __cplusplus
}
#endif

#endif /* OFFROAD_PLANNER_MESSAGES_H_ */

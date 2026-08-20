/*
 * offroad_planner_messages.h
 *
 *  Created on: 14/05/2020
 *      Author: Alberto
 */

#ifndef OFFROAD_PLANNER_MESSAGES_H_
#define OFFROAD_PLANNER_MESSAGES_H_

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef enum OFFROAD_PLANNER_FEEDBACK
{
	PLAN_OK,
	ALREADY_IN_GOAL,
	GOAL_OVER_OBSTACLE,
	START_OVER_OBSTACLE,
	TIMEOUT,
	GOAL_NOT_FOUND,
	GOAL_NOT_SET,
	OBSTACLE_DISTANCE_MAP_NOT_AVAILABLE,
	COULD_NOT_COMPUTE_GOAL_DISTANCE_MAP,
	COULD_NOT_FIND_SUITABLE_GOAL_POSE,
	OFFROAD_PLANNER_TEST
} carmen_offroad_planner_feedback_t;

#define print_offroad_planner_feedback(x) ( \
	(x == PLAN_OK)? "PLAN_OK": \
	(x == ALREADY_IN_GOAL)? "ALREADY_IN_GOAL": \
	(x == GOAL_OVER_OBSTACLE)? "GOAL_OVER_OBSTACLE": \
	(x == START_OVER_OBSTACLE)? "START_OVER_OBSTACLE": \
	(x == TIMEOUT)? "TIMEOUT": \
	(x == GOAL_NOT_FOUND)? "GOAL_NOT_FOUND": \
	(x == GOAL_NOT_SET)? "GOAL_NOT_SET": \
	(x == OBSTACLE_DISTANCE_MAP_NOT_AVAILABLE)? "OBSTACLE_DISTANCE_MAP_NOT_AVAILABLE": \
	(x == COULD_NOT_COMPUTE_GOAL_DISTANCE_MAP)? "COULD_NOT_COMPUTE_GOAL_DISTANCE_MAP": \
	(x == COULD_NOT_FIND_SUITABLE_GOAL_POSE)? "COULD_NOT_FIND_SUITABLE_GOAL_POSE": \
	(x == OFFROAD_PLANNER_TEST)? "OFFROAD_PLANNER_TEST": "" \
	)


typedef struct
{
	carmen_offroad_planner_feedback_t offroad_planner_feedback;
	int number_of_poses;
	carmen_robot_and_trailers_traj_point_t *poses;
	int pose_id;
	carmen_robot_and_trailers_traj_point_t transition_pose;
	carmen_robot_and_trailers_traj_point_t goal_pose;

    double timestamp;
    char *host;
} carmen_offroad_planner_plan_message;

#define		CARMEN_OFFROAD_PLANNER_PLAN_MESSAGE_NAME	"carmen_offroad_planner_plan_message"
#define		CARMEN_OFFROAD_PLANNER_PLAN_MESSAGE_FMT		"{int, int, <{double, double, double, int, [double:5], double, double}:2>, int, {double, double, double, int, [double:5], double, double}, {double, double, double, int, [double:5], double, double}, double, string}"

#ifdef __cplusplus
}
#endif

#endif /* OFFROAD_PLANNER_MESSAGES_H_ */

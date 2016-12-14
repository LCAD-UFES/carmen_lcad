/*
 * stehs_planner_messages.h
 *
 *  Created on: 14 de dez de 2016
 *      Author: luan
 */

#ifndef SRC_STEHS_PLANNER_STEHS_PLANNER_MESSAGES_H_
#define SRC_STEHS_PLANNER_STEHS_PLANNER_MESSAGES_H_


typedef struct {
	carmen_ackerman_traj_point_t p1, p2;
	double v, phi;
	double time;
} Edge_Struct;


typedef struct
{
	Edge_Struct *path;
	int size;
	carmen_point_t goal;
	int last_goal;
	double timestamp;
	char  *host;
} rrt_path_message;

#define RRT_PATH_NAME "rrt_path_message_name"
#define RRT_PATH_FMT "{<{{double, double, double, double, double}, {double, double, double, double, double}, double, double, double}:2>, int, {double, double, double}, int, double, string}"


#endif /* SRC_STEHS_PLANNER_STEHS_PLANNER_MESSAGES_H_ */

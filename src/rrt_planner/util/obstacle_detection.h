/*
 * obstacle_detection.h
 *
 *  Created on: 09/04/2012
 *      Author: romulo
 */

#ifndef OBSTACLE_DETECTION_H_
#define OBSTACLE_DETECTION_H_

#include <carmen/carmen.h>
#include "../model/pose.h"
#include "../model/command.h"
#include "../model/cost_map.h"
#include "../model/robot_state.h"
#include "../model/rrt_node.h"

typedef enum
{
	OBSTACLE_POSITION = 1, UNKNOWN_POSITION = -1, FREE_POSITION = 0, INVALID_POSITION = 2
} POSITION_TYPES;

class Obstacle_Detection
{
public:
	static bool is_lane(Pose &global_pose);
	static bool	is_obstacle_point(Pose &pose);
	static bool	is_obstacle_point(const int &x, const int &y);
	static bool is_obstacle(Pose &global_pose);
	static bool is_obstacle(Pose &global_pose, carmen_map_t &map, double threshold = 0.5);
	static bool is_obstacle_path(Pose &global_pose, const Command &command, double interval_time);
	static bool is_obstacle_path2(Robot_State &robot_state, const Command &command, double interval_time);
	static bool is_obstacle_path(RRT_Node &node, Command &command, double interval_time);
};

#endif /* OBSTACLE_DETECTION_H_ */

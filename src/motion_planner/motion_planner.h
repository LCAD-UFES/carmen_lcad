/*
 * motion_planner.h
 *
 *  Created on: 20/09/2012
 *      Author: romulo
 */

#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_

#include <carmen/carmen.h>
#include <carmen/behavior_selector_messages.h>
#include "motion_planner_messages.h"


#define NUM_MOTION_COMMANDS_VECTORS	10
#define	NUM_MOTION_COMMANDS_PER_VECTOR	200

#define MAX_TRAJECTORY_VECTOR_OF_POINTS_SIZE (NUM_MOTION_COMMANDS_PER_VECTOR * 10)

#define NUM_MAPS 5

void motion_planner_set_robot_pose(carmen_point_t pose, double v, double phi);
void motion_planner_set_odometry(double v, double phi);
void motion_planner_set_path(carmen_motion_planner_path_message *msg);
void motion_planner_go();
void motion_planner_stop();
void motion_planner_set_algorithm(carmen_behavior_selector_algorithm_t new_algorithm, carmen_behavior_selector_mission_t new_mission);
int motion_planner_read_parameters(int argc, char **argv);
carmen_navigator_ackerman_plan_message build_navigator_ackerman_plan_message(carmen_ackerman_motion_command_p motion_commands_vector, int num_motion_commands);

void publish_astar_path(carmen_ackerman_traj_point_t *path, int path_size, carmen_ackerman_traj_point_t robot_position);
void publish_motion_planner_path(carmen_ackerman_motion_command_t *motion_commands_vector, int num_motion_commands);
void publish_obstacle_avoider_path(carmen_ackerman_motion_command_t *motion_commands_vector, int num_motion_commands);
void publish_status(void);
void publish_plan(void);
void copy_grid_mapping_to_map_vector(carmen_mapper_map_message *grid_map, int position);
void motion_planning_obstacle_avoiding_handler();

#endif /* MOTION_PLANNER_H_ */

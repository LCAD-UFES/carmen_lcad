/*
 * behavior_selector.h
 *
 *  Created on: 28/09/2012
 *      Author: romulo
 */

#ifndef BEHAVIOR_SELECTOR_H_
#define BEHAVIOR_SELECTOR_H_

#include <carmen/carmen.h>
#include <carmen/rddf_messages.h>

struct _moving_object
{
	bool valid;
	int index;
	carmen_ackerman_traj_point_t pose;
	double timestamp;
};

typedef struct _moving_object MOVING_OBJECT;

#define MOVING_OBJECT_HISTORY_SIZE 20


void change_distance_between_waypoints_and_goals(double dist_between_waypoints, double change_goal_dist);

void behavior_selector_initialize(carmen_robot_ackerman_config_t config, double dist_between_waypoints,
		double change_goal_dist, carmen_behavior_selector_algorithm_t f_planner, carmen_behavior_selector_algorithm_t p_planner,
		double distance_to_remove_annotation_goal,int rddf_num_poses_ahead_min, int rddf_num_poses_ahead_limited_by_map);

void behavior_selector_update_robot_pose(carmen_ackerman_traj_point_t robot_pose);

void behavior_selector_update_rddf(carmen_rddf_road_profile_message *rddf_msg, int rddf_num_poses_by_velocity, double timestamp);

void behavior_selector_update_map(carmen_map_t *map);

void behavior_selector_publish_periodic_messages();

void behavior_selector_set_algorithm(carmen_behavior_selector_algorithm_t algorithm, carmen_behavior_selector_state_t state);

int behavior_selector_set_state(carmen_behavior_selector_state_t state);

int behavior_selector_set_goal_source(carmen_behavior_selector_goal_source_t goal_source);

void behavior_selector_add_goal(carmen_point_t goal);

void behavior_selector_clear_goal_list();

void behavior_selector_remove_goal();

void behavior_selector_update_annotations(carmen_rddf_annotation_message *message);

carmen_behavior_selector_algorithm_t get_current_algorithm();

void behavior_selector_get_state(carmen_behavior_selector_state_t *current_state_out, carmen_behavior_selector_algorithm_t *following_lane_planner_out,
		carmen_behavior_selector_algorithm_t *parking_planner_out, carmen_behavior_selector_goal_source_t *current_goal_source_out);

carmen_ackerman_traj_point_t *behavior_selector_get_goal_list(int *goal_list_size_out);

carmen_ackerman_traj_point_t get_robot_pose();
double get_max_v();
carmen_robot_ackerman_config_t *get_robot_config();

carmen_rddf_road_profile_message *get_last_rddf_message();

// TODO: retirar as duas funcoes deste contexto de lib e deixa-las apenas visiveis (static) no behavior_selector_main.cpp
void publish_goal_list();
void publish_current_state();

int update_goal_list(double timestamp);

#endif /* BEHAVIOR_SELECTOR_H_ */

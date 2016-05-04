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

void change_distance_between_waypoints_and_goals(double dist_between_waypoints, double change_goal_dist);

void behavior_selector_initialize(carmen_robot_ackerman_config_t config, double dist_between_waypoints, double change_goal_dist, carmen_behavior_selector_algorithm_t f_planner, carmen_behavior_selector_algorithm_t p_planner);

void behavior_selector_update_robot_pose(carmen_ackerman_traj_point_t robot_pose, int *state_updated);

void behavior_selector_update_rddf(carmen_rddf_road_profile_message *rddf_msg);

void behavior_selector_update_map(carmen_map_t *map, int *goal_list_updated);

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

void behavior_selector_get_goal_list(carmen_ackerman_traj_point_t **goal_list_out, int *goal_list_size_out, int *goal_list_index_out, double *goal_list_time_out);

carmen_ackerman_traj_point_t get_robot_pose();

// TODO: retirar as duas funcoes deste contexto de lib e deixa-las apenas visiveis (static) no behavior_selector_main.cpp
void publish_goal_list();
void publish_current_state();

#endif /* BEHAVIOR_SELECTOR_H_ */

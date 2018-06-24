/*
 * trajectory_lookup_table.h
 *
 *  Created on: Jun 22, 2016
 *      Author: alberto
 */

#ifndef MODEL_PREDICTIVE_PLANNER_H_
#define MODEL_PREDICTIVE_PLANNER_H_

#include "trajectory_lookup_table.h"

vector<vector<carmen_ackerman_path_point_t> > compute_path_to_goal(Pose *localize_pose, Pose *goal_pose,
			Command last_odometry, double max_v, carmen_behavior_selector_road_profile_message *goal_list_message);

carmen_ackerman_path_point_t move_to_front_axle(carmen_ackerman_path_point_t pose);

//double compute_distance_to_closest_obstacles(carmen_ackerman_path_point_t path_pose, double circle_radius);



#endif /* MODEL_PREDICTIVE_PLANNER_H_ */


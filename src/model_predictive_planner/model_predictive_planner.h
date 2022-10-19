/*
 * trajectory_lookup_table.h
 *
 *  Created on: Jun 22, 2016
 *      Author: alberto
 */

#ifndef MODEL_PREDICTIVE_PLANNER_H_
#define MODEL_PREDICTIVE_PLANNER_H_

vector<vector<carmen_robot_and_trailers_path_point_t> > compute_path_to_goal(carmen_robot_and_trailers_pose_t *localize_pose, Pose *goal_pose,
			Command last_odometry, double max_v, carmen_behavior_selector_path_goals_and_annotations_message *path_goals_and_annotations_message);

carmen_ackerman_path_point_t move_to_front_axle(carmen_ackerman_path_point_t pose);

//double compute_distance_to_closest_obstacles(carmen_ackerman_path_point_t path_pose, double circle_radius);

#endif /* MODEL_PREDICTIVE_PLANNER_H_ */


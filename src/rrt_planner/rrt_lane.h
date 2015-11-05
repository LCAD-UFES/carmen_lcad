/*
 * rrt_lane.h
 *
 *  Created on: 15/05/2014
 *      Author: romulo
 */

#ifndef RRT_LANE_H_
#define RRT_LANE_H_

#include "rrt.h"

class RRT_Lane : public RRT {
public:
	RRT_Lane();

	void build_rrt_path_from_robot_pose(Robot_State &robot_pose);

	void build_rrt_path();

	void compute_x_new_alternatives_and_associated_commands_and_times(vector<RRT_Node *> &x_new_alternatives_computed,
			vector<Command> &x_new_associated_commands, vector<double> &x_new_associated_commands_times,
			RRT_Node &x_near, Pose &x_rand);

	void create_sub_lane_points_vector();


	Pose random_pose_close_to_the_lane(double p_sub_lane, double radius = 1);

	Pose random_lane_pose(double radius = 1.5);

	/**
	 * Sub lane pose is a lane pose between the robot and the goal
	 */
	Pose random_sub_lane_pose();
	vector<Pose> sub_lane_points;//pontos da rua que estão entre o goal e o robo

	Cost_Function *lane_cost_functions[10];
	double lane_weights[10];
	int num_lane_cost_function;
};

#endif /* RRT_LANE_H_ */

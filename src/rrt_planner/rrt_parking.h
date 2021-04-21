/*
 * rrt_parking.h
 *
 *  Created on: 15/05/2014
 *      Author: romulo
 */

#ifndef RRT_PARKING_H_
#define RRT_PARKING_H_

#include "rrt.h"

class RRT_Parking : public RRT {
public:
	RRT_Parking();

	void build_rrt_path_from_robot_pose(Robot_State &robot_pose);

	void reuse_previous_path();

	void build_rrt_path();

	void compute_x_new_alternatives_and_associated_commands_and_times(vector<RRT_Node *> &x_new_alternatives_computed,
			vector<Command> &x_new_associated_commands, vector<double> &x_new_associated_commands_times,
			RRT_Node &x_near, Pose &x_rand);


	/**
	 * Set the max distance in meters of grad poses
	 */
	void set_max_distance_grad(double distance);

	/**
	 * If the distance between the node and the goal is lesser than distance_near the node is close to robot
	 * distance_near in meters
	 */
	void set_distance_near(double distance_near);



	/**
	 * k numero de tentativas
	 */
	RRT_Node *connect_grad(RRT_Node *near, int *status, int k = 15);

	Pose get_gradient_node(Pose &p, double dist = 20);



	double max_dist_grad;//distancia maxima da pose de gradiente a frente
	double gradient_probability; //probabilidade do utility map ser usado para guiar extend e connect

	Cost_Function *parking_cost_functions[10];
	double parking_weights[10];
	int num_parking_cost_function;
	double distance_near; //if the distance between robot and goal is lesser than distance near, than the robot is close to the goal
};

#endif /* RRT_PARKING_H_ */

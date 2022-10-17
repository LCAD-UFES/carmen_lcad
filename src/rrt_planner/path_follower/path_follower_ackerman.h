/*
 * PathFollowerAckerman.h
 *
 *  Created on: 11/04/2012
 *      Author: romulo
 */

#ifndef PATHFOLLOWERACKERMAN_H_
#define PATHFOLLOWERACKERMAN_H_
using namespace std;

#include "path_follower.h"
#include "../model/command.h"
#include <list>


class Path_Follower_Ackerman : public Path_Follower
{
public:
	void build_and_send_refined_path();
	void go();
	void stop();
	void set_path(list<RRT_Path_Edge> path);
	list<RRT_Path_Edge> &get_path();
	void set_path_lost_condition(double distance_threshold, double theta_threshold);
	void publish_path_follower_single_motion_command(double v, double phi, double timestamp);
	void publish_path_follower_motion_commands(carmen_robot_and_trailers_motion_command_t *commands, int num_commands, double timestamp);

	/**
	 * Verify if the car advanced on the path
	 * If so, update the path
	 */
	void update_path();

private:
	void build_and_send_robot_motion_command_vector();
	void send_command(Command c, double time = 10);
	double get_current_max_velocity(double v, Robot_State &robot_state, list<RRT_Path_Edge>::iterator edge); // can reduce velocity if the is a problem
	void compute_motion_command_vector(int &motion_command_size,
			Robot_State current_state, Command command, double remaining_command_time, double fixed_time);//, list<RRT_Path_Edge>::iterator edge);
	int handle_empty_rrt_path(int motion_command_size);

	double distance_threshold, theta_threshold;

	double time_robot_lost; //the time that the robot lost the path

	bool verify_path_collision;

	Robot_State path_pose; // Intermediary robot_state on the path
	double spent_time; //time to reach the robot_state on the path
	double total_distance, traveled_distance;

public:
	list<RRT_Path_Edge> path;
	bool robot_lost; //robot is lost on the path, reduce velocity ...

	static int use_obstacle_avoider;
};

#endif /* PATHFOLLOWERACKERMAN_H_ */

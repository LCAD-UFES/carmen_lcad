/*
 * cost_function.h
 *
 *  Created on: 22/02/2013
 *      Author: romulo
 */

#ifndef COST_FUNCTION_H_
#define COST_FUNCTION_H_

#include "../model/robot_state.h"
#include "../model/command.h"
#include "../model/global_state.h"
#include "util.h"

class Cost_Function {
public:
	virtual ~Cost_Function(){
}

virtual void set_near(const Robot_State& near) {
	this->near = near;
}

virtual void set_distance_interval(const double& distance_interval)
{
	this->distance_interval = distance_interval;
}

virtual void set_new_robot_state(const Robot_State& newRobotState) {
	new_robot_state = newRobotState;
}

virtual void set_time(double time) {
	this->time = time;
}

virtual void set_velocity(double v) {
	command.v = v;
}

virtual void set_phi(double phi) {
	command.phi = phi;
}

virtual void set_rand_pose(const Pose& rand_pose)
{
	this->rand_pose = rand_pose;
}

virtual double get_cost() {
	return cost;
}

protected:
Pose rand_pose;
Robot_State near;
Command command;
double time;
Robot_State new_robot_state;
double cost;
double distance_interval;
};

class Distance_Cost_Function : public Cost_Function {
public:
	double get_cost();
};

class Normalized_Distance_Cost_Function : public Cost_Function {
public:
	double get_cost();

	void set_distance_interval(const double& distance_interval);

private:
	double min_distance;
	double max_distance;
};

class Velocity_Cost_Function : public Cost_Function {
public:
	void set_velocity(double v) {
		command.v = v;
		cost = 1.0 - exp(-0.08 * pow(GlobalState::robot_config.max_vel - command.v, 2));//pow(GlobalState::robot_config.max_vel - command.v, 2) / (command.v * command.v);
	}
};

class Lane_Cost_Function : public Cost_Function {
public:
	double get_cost();

	static double get_cost(const Robot_State robot_state);
};

class Rear_Motion_Cost_Function : public Cost_Function {
public:
	void set_velocity(double v) {
		command.v = v;
		cost = command.v < 0.0 ? 1.0 : 0.0;
	}
};

class Change_Direction_Cost_Function : public Cost_Function {
public:
	void set_near(const Robot_State& near) {
		this->near = near;
		direction = Util::signal(near.v_and_phi.v);
	}

	void set_velocity(double v) {
		command.v = v;
		cost = (direction != Util::signal(v)) ? 1.0 : 0.0;

		if (this->near.v_and_phi.v == 0.0 || v == 0.0)
			cost = 0.0;
	}

private:
	double direction;
};

class Align_Cost_Function : public Cost_Function {
public:
	double get_cost();
};

class Obstacle_Cost_Function : public Cost_Function {
public:
	double get_cost();
};

class Phi_Change_Cost_Function : public Cost_Function {
public:
	double get_cost();
};

class Obstacle_Cost_Function2 : public Cost_Function {
public:
	double get_cost();

	static double get_cost(const Pose &global_pose, const carmen_map_t &map);
};

#endif /* COST_FUNCTION_H_ */

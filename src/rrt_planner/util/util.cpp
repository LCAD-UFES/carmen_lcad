/*
 * util.cpp
 *
 *  Created on: 01/02/2012
 *      Author: rradaelli
 */

#include "util.h"
#include <carmen/carmen.h>
#include <sys/time.h>
#include "../model/global_state.h"
#include "publisher_util.h"
#include "obstacle_detection.h"


bool Util::is_valid_position(const int &x, const int &y, const carmen_map_config_t &map_config)
{
	return x >= 0 && x < map_config.x_size && y >= 0 && y < map_config.y_size;
}

bool Util::is_valid_position(const int &x, const int &y)
{
	return is_valid_position(x, y, GlobalState::cost_map.config);
}

bool Util::is_valid_position(const Pose &pose, const carmen_map_config_t &map_config)
{
	return is_valid_position(round(pose.x), round(pose.y), map_config);
}

bool Util::is_valid_position(const Pose &pose)
{
	return is_valid_position(round(pose.x), round(pose.y), GlobalState::cost_map.config);
}

double Util::heading(const Pose &pose, const Pose &target_pose)
{
	double target_theta = fabs(carmen_normalize_theta(atan2(pose.y - target_pose.y, pose.x - target_pose.x) - M_PI - pose.theta));

	return target_theta / M_PI;
}

bool Util::is_behind(const Pose &p1, const Pose &p2)
{
	return fabs(carmen_normalize_theta(atan2(p1.y - p2.y, p1.x - p2.x) - M_PI - p1.theta)) > M_PI_2;
}

double Util::heading2(const Pose &pose, const Pose &target_pose)
{
	double alpha, target_theta, theta_diff;
	int	   is_goal_behind, is_opposite_side;

	alpha = carmen_normalize_theta(atan2(pose.y - target_pose.y, pose.x - target_pose.x) - M_PI);
	target_theta	 = fabs(carmen_normalize_theta(alpha - pose.theta));
	is_goal_behind	 = target_theta > M_PI_2;
	theta_diff		 = fabs(carmen_normalize_theta(pose.theta - target_pose.theta));
	is_opposite_side = theta_diff > M_PI_2;

	return fabs(is_opposite_side - fabs(is_goal_behind - (target_theta / M_PI)));
}

int Util::signalz(double num)
{
	if (num > 0)
	{
		return 1;
	}
	else if (num < 0)
	{
		return -1;
	}

	return 0;
}

int Util::signal(double num)
{
	return num >= 0 ? 1 : -1;
}

/**
 * Random double between 0 and 1
 */
double Util::normalized_random()
{
	return ((double)rand()) / ((double)RAND_MAX);
}

Pose Util::random_pose()
{
	Pose p;

	p.x		= rand() % GlobalState::cost_map.config.x_size;
	p.y		= rand() % GlobalState::cost_map.config.y_size;
	p.theta = carmen_normalize_theta(carmen_degrees_to_radians((rand() % 360) - 180));

	p = Util::to_global_pose(p);

	return p;
}

Command Util::random_command()
{
	Command c;
	c.v	  = (normalized_random() * GlobalState::robot_config.max_vel * 2) - GlobalState::robot_config.max_vel;
	c.phi = (normalized_random() * GlobalState::robot_config.max_phi * 2) - GlobalState::robot_config.max_phi;

	return c;
}

double Util::get_time()
{
	struct timeval t;

	gettimeofday(&t, NULL);
	return (t.tv_sec * 1000000 + t.tv_usec) * 0.000001;
}

Pose Util::to_map_pose(carmen_point_t world_pose)
{
	Pose p;

	p.theta = world_pose.theta;
	p.x		= (world_pose.x - GlobalState::cost_map.config.x_origin) / GlobalState::cost_map.config.resolution;
	p.y		= (world_pose.y - GlobalState::cost_map.config.y_origin) / GlobalState::cost_map.config.resolution;

	return p;
}

Pose Util::to_map_pose(const Pose &world_pose, const carmen_map_config_t &map_config)
{
	Pose p;

	p.theta = world_pose.theta;
	p.x		= (world_pose.x - map_config.x_origin) / map_config.resolution;
	p.y		= (world_pose.y - map_config.y_origin) / map_config.resolution;

	return p;
}

Pose Util::to_map_pose(const Pose &world_pose)
{
	return to_map_pose(world_pose, GlobalState::cost_map.config);
}

carmen_point_t Util::to_world_pose(Pose map_pose)
{
	carmen_point_t p;

	p.theta = map_pose.theta;
	p.x		= map_pose.x * GlobalState::cost_map.config.resolution + GlobalState::cost_map.config.x_origin;
	p.y		= map_pose.y * GlobalState::cost_map.config.resolution + GlobalState::cost_map.config.y_origin;

	return p;
}

Pose Util::to_global_pose(const Pose &map_pose)
{
	return to_global_pose(map_pose, GlobalState::cost_map.config);
}

Pose Util::to_global_pose(const Pose &map_pose, const carmen_map_config_t &map_config)
{
	Pose p;

	p.theta = map_pose.theta;
	p.x		= map_pose.x * map_config.resolution + map_config.x_origin;
	p.y		= map_pose.y * map_config.resolution + map_config.y_origin;

	return p;
}

double Util::to_meter(const double &map_unit)
{
	return map_unit * GlobalState::cost_map.config.resolution;
}

double Util::to_map_unit(const double &meter)
{
	return to_map_unit(meter, GlobalState::cost_map.config);
}

double Util::to_map_unit(const double &meter, const carmen_map_config_t &map_config)
{
	return meter / map_config.resolution;
}

bool Util::can_reach(Pose robot, Pose goal)
{
	double theta_diff;
	bool   goal_reachable;

	theta_diff = goal.theta - robot.theta;

	goal_reachable = fabs(theta_diff) < carmen_degrees_to_radians(70);

	//printf("%f %f %d\n", carmen_radians_to_degrees(theta_diff), heading(robot, goal), goal_reachable);

	return goal_reachable;
}


Pose
Util::convert_to_pose(const carmen_point_t carmen_pose, double beta)
{
	Pose p;

	p.x = carmen_pose.x;
	p.y = carmen_pose.y;
	p.theta = carmen_pose.theta;
	p.beta = beta;

	return p;
}


carmen_point_t
Util::convert_to_carmen_point_t(const Pose pose)
{
	carmen_point_t p;

	p.x = pose.x;
	p.y = pose.y;
	p.theta = pose.theta;

	return p;
}

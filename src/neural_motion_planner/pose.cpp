/*
 * pose.cpp
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */

#include "pose.h"
#include <math.h>
#include "model/global_state.h"
#include "rrt_node.h"
//#include "../rs.h"

Pose::Pose()
{
	x	  = 0;
	y	  = 0;
	theta = 0;
}


bool Pose::operator==(const Pose &pose)
{
	return fabs(x - pose.x) < 0.001 && fabs(y - pose.y) < 0.001 && fabs(theta - pose.theta) < 0.001;
}

bool Pose::operator!=(const Pose &pose)
{
	return !(*this == pose);
}


double Pose::distance(const Pose &p)
{
	return sqrt(
			   (x - p.x) * (x - p.x) +
			   (y - p.y) * (y - p.y));
}

double Pose::distance(const RRT_Node &node)
{
	return distance(node.robot_state.pose);
}

double Pose::distance(const RRT_Node *node)
{
	return distance(node->robot_state.pose);
}

double Pose::get_theta_diff(const Pose &p)
{
	return fabs(carmen_normalize_theta(carmen_normalize_theta(theta - p.theta)));
}

double Pose::get_theta_diff(double t)
{
	return fabs(carmen_normalize_theta(carmen_normalize_theta(theta - t)));
}

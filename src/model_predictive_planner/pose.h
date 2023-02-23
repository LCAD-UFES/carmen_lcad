/*
 * pose.h
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */

#ifndef POSE_H_
#define POSE_H_

#include <carmen/carmen.h>

class RRT_Node;

class Pose
{
public:
	Pose();
	double distance(const Pose &p);
	double distance(const RRT_Node &node);
	double distance(const RRT_Node *node);
	double get_theta_diff(const Pose &p);
	double get_theta_diff(double theta);
	bool operator ==(const Pose &pose);
	bool operator !=(const Pose &pose);

	double x;
	double y;
	double theta;
//	double beta;
	double trailer_theta[MAX_NUM_TRAILERS];
};

#endif /* POSE_H_ */

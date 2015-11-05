/*
 * PathFollower.h
 *
 *  Created on: 11/04/2012
 *      Author: romulo
 */

#ifndef PATHFOLLOWER_H_
#define PATHFOLLOWER_H_

#include "../rrt.h"

class Path_Follower
{
public:
	virtual void build_and_send_refined_path() = 0;
	virtual void go()	= 0;
	virtual void stop() = 0;
};

#endif /* PATHFOLLOWER_H_ */

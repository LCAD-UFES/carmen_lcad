/*
 * utiliIy.h
 *
 *  Created on: 19/04/2012
 *      Author: romulo
 */

#ifndef UTILIIY_H_
#define UTILIIY_H_
using namespace std;
#include <queue>
#include <vector>
#include "pose.h"
#include <carmen/carmen.h>

typedef struct
{
	Pose   pose;
	double cost;
} Grad_State;

typedef struct
{
	carmen_map_config_t config;
	double *costs;
} Gradient_Cost_Map;

class Compare_Utility_State
{
public:
	bool operator()(const Grad_State &a, const Grad_State &b) const
	{
		return a.cost < b.cost;
	}
};

typedef priority_queue<Grad_State, vector<Grad_State>, Compare_Utility_State> utility_priority_queue;

#endif /* UTILIIY_H_ */

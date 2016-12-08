#ifndef STEHS_STATE_NODE_HPP
#define STEHS_STATE_NODE_HPP

#include <vector>
#include <carmen/carmen.h>

#include "Helpers/wrap2pi.hpp"

typedef carmen_point_t Pose;

typedef carmen_ackerman_traj_point_t State;

#define MIN_THETA_DISTANCE 0.261799 // 15 degrees to radians
#define MIN_POS_DISTANCE 0.2 // the carmen grid map resolution

class StateNode
{

public:

	// the current state
	State state;

	// the (time) cost from start to current state
	double g;

	// the total (time) cost, g + h
	double f;

	// the (time) cost from current state to goal
	double h;

	// step size
	double step_size;

	// the parent node
	StateNode *parent;

	// the theta weight

	// basic constructor
	StateNode(const State &s, double g_, double h_, StateNode *p);

	// basic constructor
	//StateNode(double x, double y, double radius, double g_, double f_, StateNode *p);

	// the copy constructor
	StateNode(const StateNode& sn);

	// basic destructor
	~StateNode();

	// the distance between two states
	bool Equals(const StateNode &sn, double k);

	// < operator overloading, for priority queue compare purpose
	bool operator<(const StateNode &sn);

	// Assignment operator
	void operator=(const StateNode &sn);

};

typedef StateNode* StateNodePtr;

// define a comparator class
class StateNodePtrComparator {

public:

	// operator overloading
	bool operator() (StateNodePtr a, StateNodePtr b)
	{

//		if(std::fabs(a->f - b->f) < 0.2 )
//		{
//			return (a->h > b->h);
//		}
//		else
//		{
//			return (a->f > b->f);
//		}
		// the default c++ stl is a max heap, so wee need to invert here
		return (a->f > b->f);
	}
};

#endif

#ifndef STEHS_CIRCLE_NODE_HPP
#define STEHS_CIRCLE_NODE_HPP

#include <vector>
#include "Circle.hpp"

class CircleNode
{

public:

	// the current circle reference
	Circle circle;

	// the g cost value
	double g;

	// the total cost, included heuristic value
	double f;

	// the parent node
	CircleNode *parent;

	// basic constructor
	CircleNode(const Circle &c, double g_, double f_, CircleNode *p);

	// basic constructor
	CircleNode(double cx, double cy, double radius, double g_, double f_, CircleNode *p);

	// the copy constructor
	CircleNode(const CircleNode& cn);

	// basic destructor
	~CircleNode();

	// < operator overloading, for priority queue compare purpose
	bool operator<(const CircleNode &cn);

	// Assignment operator
	void operator=(const CircleNode &cn);

};

typedef CircleNode* CircleNodePtr;

// define a comparator class
class CircleNodePtrComparator {

public:

	// operator overloading
	bool operator() (CircleNodePtr a, CircleNodePtr b) {

	// the default c++ stl is a max heap, so wee need to invert here
	return a->f > b->f;

	}

};

#endif

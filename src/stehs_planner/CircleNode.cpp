#include "CircleNode.hpp"

// basic constructor
CircleNode::CircleNode(const Circle &c, double g_, double f_, CircleNode *p) :
	circle(c), g(g_), f(f_), parent(p) {}

// the copy constructor
CircleNode::CircleNode(const CircleNode& cn) :
	circle(cn.circle), g(cn.g), f(cn.f), parent(nullptr) {}

// basic destructor
CircleNode::~CircleNode() {

	// reset the parent node to nullptr
	parent = nullptr;

}

// < operator overloading, for priority queue compare purpose
bool CircleNode::operator<(const CircleNode &cn) {

	return f < cn.f;

}

// Assignment operator
// copy the node values
void CircleNode::operator=(const CircleNode &cn) {

	// the circle position
	circle.x = cn.circle.x;

	circle.y = cn.circle.y;

	// the circle radius
	circle.radius = cn.circle.radius;

	// the cost
	g = cn.g;

	// the heuristic cost
	f = cn.f;

	// the parent pointer
	parent = cn.parent;

}

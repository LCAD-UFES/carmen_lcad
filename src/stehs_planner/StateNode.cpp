#include "StateNode.hpp"

// basic constructor
StateNode::StateNode(const State &s, double g_, double h_, StateNode *p) :
	g(g_), f(g_ + h_), h(h_), parent(p) {

	state.x = s.x;
	state.y = s.y;
	state.theta = s.theta;
	state.v = s.v;
	state.phi = s.phi;
}

// basic constructor
//StateNode::StateNode(double x, double y, double radius, double g_, double f_, StateNode *p) :
//	circle(cx, cy, radius), g(g_), f(f_), parent(p)
//{}


// the copy constructor
StateNode::StateNode(const StateNode& sn) :
	 g(sn.g), f(sn.f), h(sn.h), parent(sn.parent) {

	state.x = sn.state.x;
	state.y = sn.state.y;
	state.theta = sn.state.theta;
	state.v = sn.state.v;
	state.phi = sn.state.phi;
}

// basic destructor
StateNode::~StateNode() {

	// reset the parent node to nullptr
	parent = nullptr;

}

// the distance between two states
bool StateNode::Equals(const StateNode &sn, double k) {

	// the displacement
	double dpos = std::sqrt(std::pow(state.x - sn.state.x, 2) + std::pow(state.y - sn.state.y, 2));

	// the angle distance
	double dtheta = mrpt::math::angDistance<double>(state.theta, sn.state.theta);

	if (MIN_POS_DISTANCE > k*dpos && MIN_THETA_DISTANCE > k*dtheta)
		return true;

	return false;

}

// < operator overloading, for priority queue compare purpose
bool StateNode::operator<(const StateNode &sn) {

	return f < sn.f;

}

// Assignment operator
// copy the node values
void StateNode::operator=(const StateNode &sn) {

	state.x = sn.state.x;
	state.y = sn.state.y;
	state.theta = sn.state.theta;
	state.v = sn.state.v;
	state.phi = sn.state.phi;

	// the cost
	g = sn.g;

	// the heuristic and cumulative cost
	f = sn.f;

	// the heuristic value
	h = sn.h;

	// the parent pointer
	parent = sn.parent;
}


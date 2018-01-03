#include "track.h"

#include "point.h"
#include "random.h"

namespace virtual_scan
{


Track::Track():
	id(this)
{
	// Nothing to do.
}


void Track::pop_back(int r)
{
	erase(begin() + r, end());
}


void Track::pop_back(int r, Track &that)
{
	that.insert(that.end(), begin() + r, end());
	pop_back(r);
}


void Track::pop_front(int r)
{
	erase(begin(), begin() + r);
}


bool Track::is_mergeable(const Track &that) const
{
	const Node *last_node = this->back().node;
	const Node::Edges &parents = that.front().node->parents;
	for (auto i = parents.begin(), n = parents.end(); i != n; ++i)
		if (*i == last_node)
			return true;

	return false;
}


void Track::merge(Track &that)
{
	that.pop_back(-1, *this);
}


int Track::diffuse()
{
	std::normal_distribution<> normal;

	int n = random_int(0, size());
	ObstaclePose &pose = at(n);

	Pose delta(normal(RD), normal(RD), normal(RD));
	pose.global += delta;
	pose.local += delta;

	return n;
}


} // namespace virtual_scan

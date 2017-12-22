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


ObstaclePose &Track::operator[] (int index)
{
	return poses[index];
}


const ObstaclePose &Track::operator[] (int index) const
{
	return poses[index];
}


ObstaclePose &Track::back()
{
	return poses.back();
}


const ObstaclePose &Track::back() const
{
	return poses.back();
}


size_t Track::size() const
{
	return poses.size();
}


void Track::push_front(Node *node)
{
	poses.emplace_front(node); // poses.push_front(Obstacle(node));
}


void Track::push_back(Node *node)
{
	poses.emplace_back(node);
}


Node *Track::at_node(int index)
{
	return poses.at(index).node;
}


const Node *Track::at_node(int index) const
{
	return poses.at(index).node;
}


Node *Track::front_node()
{
	return poses.front().node;
}


Node *Track::back_node()
{
	return poses.back().node;
}


const Node *Track::front_node() const
{
	return poses.front().node;
}


const Node *Track::back_node() const
{
	return poses.back().node;
}


void Track::pop_back(int r)
{
	poses.erase(poses.begin() + (r + 1), poses.end());
}


void Track::pop_back(int r, Track &that)
{
	for (int i = r + 1, n = poses.size(); i < n; i++)
		that.poses.push_back(poses[i]);

	poses.erase(poses.begin() + (r + 1), poses.end());
}


void Track::pop_front(int r)
{
	poses.erase(poses.begin(), poses.begin() + r);
}


bool Track::is_mergeable(const Track &that) const
{
	const Node *last_node = this->back_node();
	const Node::Edges &parents = that.front_node()->parents;
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
	ObstaclePose &pose = poses[n];

	Pose delta(normal(RD), normal(RD), normal(RD));
	pose.global += delta;
	pose.local += delta;

	return n;
}


} // namespace virtual_scan

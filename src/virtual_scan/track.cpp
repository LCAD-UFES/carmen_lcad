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


size_t Track::size() const
{
	return poses.size();
}


void Track::push_front(virtual_scan_graph_node_t *node)
{
	poses.emplace_front(node); // poses.push_front(Obstacle(node));
}


void Track::push_back(virtual_scan_graph_node_t *node)
{
	poses.emplace_back(node);
}


virtual_scan_graph_node_t *Track::at_node(int index)
{
	return poses.at(index).node;
}


const virtual_scan_graph_node_t *Track::at_node(int index) const
{
	return poses.at(index).node;
}


virtual_scan_graph_node_t *Track::front_node()
{
	return poses.front().node;
}


virtual_scan_graph_node_t *Track::back_node()
{
	return poses.back().node;
}


const virtual_scan_graph_node_t *Track::front_node() const
{
	return poses.front().node;
}


const virtual_scan_graph_node_t *Track::back_node() const
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
	const virtual_scan_graph_node_t *last_node = this->back_node();
	const virtual_scan_elements_t &parents = that.front_node()->parents;

	for (int i = 0, n = parents.num_pointers; i < n; i++)
	{
		virtual_scan_graph_node_t *parent = (virtual_scan_graph_node_t*) parents.pointers[i];
		if (parent == last_node)
			return true;
	}

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

	pose.x += normal(RD);
	pose.y += normal(RD);
	pose.theta = carmen_normalize_theta(pose.theta + normal(RD));

	return n;
}


} // namespace virtual_scan

#include "track.h"

#include "point.h"
#include "random.h"

namespace virtual_scan
{


Track::~Track()
{
	for (int i = 0, n = poses.size(); i < n; i++)
		poses[i]->complete_sub_graph->selected = 0; //poses[i].graph_node->complete_sub_graph->selected = 0;
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
	return poses.at(index).graph_node;
}


virtual_scan_graph_node_t *Track::front_node()
{
	return poses.front().graph_node;
}


virtual_scan_graph_node_t *Track::back_node()
{
	return poses.back().graph_node;
}


const virtual_scan_graph_node_t *Track::front_node() const
{
	return poses.front().graph_node;
}


const virtual_scan_graph_node_t *Track::back_node() const
{
	return poses.back().graph_node;
}


void Track::pop_back(int r)
{
	for (int i = r + 1, n = poses.size(); i < n; i++)
		poses[i]->complete_sub_graph->selected = 0;

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
	for (int i = 0; i < r; i++)
		poses[i]->complete_sub_graph->selected = 0;

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


void Track::push_view(int t, const carmen_point_t &globalpos, std::vector<ObstacleView> &w) const
{
	const ObstaclePose &pose = poses[t];
	Rectangle rectangle(
		pose.graph_node->box_model.width,
		pose.graph_node->box_model.length,
		project_pose(pose, globalpos)
	);

	std::pair<double, double> angles = rectangle.obstruction();

	// If the obstacle lies on the border between quadrants 2 and 3,
	// produce two views of it: one from quadrant 2 onwards, the other
	// from 3 backwards. This is necessary to correctly match reading
	// rays to obstacles.
	if (angles.first < -M_PI_2 && angles.second > M_PI_2)
	{
		w.emplace_back(rectangle, std::make_pair(angles.second - 2.0 * M_PI, angles.first));
		w.emplace_back(rectangle, std::make_pair(angles.second, 2.0 * M_PI + angles.first));
	}
	else
		w.emplace_back(rectangle, angles);
}


double Track::P_L(double lambda_L, int T)
{
	// f(x) = exp(lambda_L * x)
	// F(x) = exp(lambda_L * x) / lambda_L
	// P_L = f(x) / int_0^T f(x) dx

	return (lambda_L * exp(lambda_L * size())) / (exp(lambda_L * T) - 1);
}


double Track::P_T() const
{
	// TODO: implement function.
	return 1.0;
}


} // namespace virtual_scan

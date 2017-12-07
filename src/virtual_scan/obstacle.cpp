#include "obstacle.h"

#include "parameters.h"

#include <cmath>
#include <limits>

namespace virtual_scan
{


ObstaclePose::ObstaclePose()
{
	graph_node = NULL;
	x = 0.0;
	y = 0.0;
	theta = 0.0;
}


ObstaclePose::ObstaclePose(virtual_scan_graph_node_t *graph_node):
	graph_node(graph_node),
	x(graph_node->box_model.x),
	y(graph_node->box_model.y),
	theta(graph_node->box_model.theta)
{
	// Nothing to do.
}


virtual_scan_graph_node_t *ObstaclePose::operator -> ()
{
	return(graph_node);
}


ObstacleView::ObstacleView():
	range(std::make_pair(0.0, 0.0))
{
	// Nothing to do.
}


ObstacleView::ObstacleView(const Rectangle &rectangle, const std::pair<double, double> &angles):
	Rectangle(rectangle),
	range(angles)
{
	// Nothing to do.
}


bool ObstacleView::operator < (const ObstacleView &that) const
{
	return this->range.first < that.range.first;
}


bool ObstacleView::operator < (const carmen_point_t &point) const
{
	return this->range.second < point.theta;
}


bool ObstacleView::operator > (const carmen_point_t &point) const
{
	return this->range.first > point.theta;
}


double ObstacleView::P_M1(const carmen_point_t &p) const
{
	Line ray({p.x, p.y});

	double d_min = std::numeric_limits<double>::max();
	carmen_position_t c = {d_min, d_min};

	// Find the obstacle point closest to the observer alongside the ray.
	for (int i = 0, n = sides.size(); i < n; i++)
	{
		std::pair<double, double> crossing = ray.crosspoint(sides[i]);
		if (crossing.first < d_min && 0 <= crossing.second && crossing.second <= 1.0)
		{
			d_min = crossing.first;
			c = ray(d_min);
		}
	}

	double d_n = DIST2D(p, c);

	return LAMBDA_1 * std::exp(-LAMBDA_1 * d_n);
}


} // namespace virtual_scan

#include "obstacle.h"

#include "parameters.h"
#include "point.h"

#include <cmath>
#include <limits>

namespace virtual_scan
{


ObstaclePose::ObstaclePose():
	node(NULL)
{
	// Nothing to do.
}


ObstaclePose::ObstaclePose(Node *node):
	node(node),
	global(node->pose.global),
	local(node->pose.local)
{
	node->select();
}


ObstaclePose::~ObstaclePose()
{
	node->deselect();
}


ObstacleView::ObstacleView():
	range(std::make_pair(0.0, 0.0))
{
	// Nothing to do.
}


ObstacleView::ObstacleView(const ObstaclePose &pose):
	Rectangle(
		pose.node->model->width,
		pose.node->model->length,
		pose.local
	)
{
	std::vector<double> angles;
	double a = std::numeric_limits<double>::max();
	double b = std::numeric_limits<double>::lowest();
	for (int i = 0, m = corners.size(), n = sides.size(); i < m; i++)
	{
		const PointXY &corner = corners[i];

		bool obstructed = false;
		for (int j = 0; j < n && !obstructed; j++)
			obstructed = sides[j].obstructs(corner);

		if (!obstructed)
		{
			// Record the minimum and maximum angles of corners
			// that have a clear view of the observer.
			double o = angle(corner);
			a = std::min(a, o);
			b = std::max(b, o);

			// Also keep negative angles for possible
			// later use (see below).
			if (o < 0.0)
				angles.push_back(o);
		}
	}

	// If the rectangle lies on the border between quadrants 2 and 3,
	// correct the start angle to be the highest negative angle (i.e.
	// the farthest from the quadrant border), then convert the range
	// to [0, 2pi) representation.
	if (a < -M_PI_2 && b > M_PI_2)
	{
		for (int i = 0, n = angles.size(); i < n; i++)
			a = std::max(a, angles[i]);

		range = std::make_pair(b, a + 2.0 * M_PI);
	}
	else
		range = std::make_pair(a, b);
}


bool ObstacleView::operator < (const ObstacleView &that) const
{
	return this->range.first < that.range.first;
}


bool ObstacleView::operator < (const PointOD &point) const
{
	// If the obstacle is over the border between quadrants 2 and 3
	// and the ray is on quadrant 3, convert the end range angle to
	// [-pi, pi) representation.
	if (range.second > M_PI && point.o < -M_PI_2)
		return (range.second - 2.0 * M_PI) < point.o;

	// Otherwise, check inequality as usual.
	return range.second < point.o;
}


bool ObstacleView::operator > (const PointOD &point) const
{
	// If the obstacle is over the border between quadrants 2 and 3
	// and the ray is on quadrant 3, convert the start range angle to
	// [-2pi, 0) representation.
	if (range.second > M_PI && point.o < -M_PI_2)
		return (range.first - 2.0 * M_PI) > point.o;

	// Otherwise, check inequality as usual.
	return range.first > point.o;
}


double ObstacleView::distance(const PointXY &p) const
{
	Line ray(p);

	double d_min = std::numeric_limits<double>::max();
	PointXY c(d_min, d_min);

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

	return virtual_scan::distance(p, c);
}


} // namespace virtual_scan

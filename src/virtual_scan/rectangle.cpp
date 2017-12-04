#include "rectangle.h"

#include "point.h"

#include <limits>

namespace virtual_scan
{


Rectangle::Rectangle():
	pose({0, 0, 0})
{
	// Nothing to do.
}


inline carmen_position_t make_corner(double x, double y, const carmen_point_t &pose)
{
	carmen_position_t c = {x, y};
	return shift(rotate(c, pose.theta), pose);
}


Rectangle::Rectangle(double width, double length, const carmen_point_t &pose)
{
	this->pose = pose;

	double w_2 = 0.5 * width;
	double l_2 = 0.5 * length;
	corners.push_back(make_corner(-l_2, -w_2, pose));
	corners.push_back(make_corner(l_2, -w_2, pose));
	corners.push_back(make_corner(l_2, w_2, pose));
	corners.push_back(make_corner(-l_2, w_2, pose));

	// Compute the obstacle's side lines.
	for (int i = 0, n = corners.size(); i < n; i++)
		sides.emplace_back(corners[i], corners[(i + 1) % n]);
}


std::pair<double, double> Rectangle::obstruction() const
{
	// Record the angles of the corners that have a clear view of the observer.
	std::vector<double> angles;
	double a = std::numeric_limits<double>::max();
	double b = std::numeric_limits<double>::lowest();
	for (int i = 0, m = corners.size(), n = sides.size(); i < m; i++)
	{
		const carmen_position_t &corner = corners[i];

		bool obstructed = false;
		for (int j = 0; j < n && !obstructed; j++)
			obstructed = sides[j].obstructs(corner);

		if (!obstructed)
		{
			double o = angle(corner);
			a = std::min(a, o); // Record the minimum and
			b = std::max(b, o); // maximum angles as well.
			angles.push_back(o);
		}
	}

	// If the rectangle lies on the border between quadrants 2 and 3,
	// correct the start angle to be the highest negative angle (i.e.
	// the farthest from the quadrant border).
	if (a < -M_PI_2 && b > M_PI_2)
	{
		for (int i = 0, n = angles.size(); i < n; i++)
		{
			double o = angles[i];
			if (o < 0)
				a = std::max(a, o);
		}
	}

	return std::make_pair(a, b);
}


} // namespace virtual_scan

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


Rectangle::Rectangle(double width, double length, const carmen_point_t &pose):
	pose(pose)
{
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


} // namespace virtual_scan

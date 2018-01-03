#include "rectangle.h"

#include "point.h"

#include <limits>

namespace virtual_scan
{


Rectangle::Rectangle()
{
	// Nothing to do.
}


inline PointXY make_corner(double x, double y, const Pose &pose)
{
	PointXY c(x, y);
	return shift(rotate(c, pose.o), pose);
}


Rectangle::Rectangle(double width, double length, const Pose &pose):
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

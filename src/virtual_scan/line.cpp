#include "line.h"

#include <cmath>

namespace virtual_scan
{

Line::Line()
{
	// Nothing to do.
}


Line::Line(const PointXY &point):
	p1(0, 0),
	p2(point)
{
	// Nothing to do.
}


Line::Line(const PointXY &p1, const PointXY &p2):
	p1(p1),
	p2(p2)
{
	// Nothing to do.
}


PointXY Line::operator () (double t) const
{
	return PointXY(p1.x + t * p2.x, p1.y + t * p2.y);
}


std::pair<double, double> Line::crosspoint(const Line &that) const
{
	double x_00 = this->p1.x;
	double y_00 = this->p1.y;
	double x_01 = this->p2.x;
	double y_01 = this->p2.y;

	double x_10 = that.p1.x;
	double y_10 = that.p1.y;
	double x_11 = that.p2.x;
	double y_11 = that.p2.y;

	// Will be 0 if lines are parallel, in which case the cross values will be nan.
	double d = x_11 * y_01 - x_01 * y_11;

	return std::make_pair(
		((x_00 - x_10) * y_11 - (y_00 - y_10) * x_11) / d,
		((x_00 - x_10) * y_01 - (y_00 - y_10) * x_01) / d
	);
}


double Line::distance(const PointXY &point) const
{
	double x_0 = point.x;
	double y_0 = point.y;
	double x_1 = p1.x;
	double y_1 = p1.y;
	double x_2 = p2.x;
	double y_2 = p2.y;

	double dx = x_2 - x_1;
	double dy = y_2 - y_1;
	double dx2 = dx * dx;
	double dy2 = dy * dy;

	// See: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
	return std::abs(dy * x_0 - dx * y_0 + x_2 * y_1 - y_2 * x_1) / std::sqrt(dx2 * dy2);
}


bool Line::obstructs(const PointXY &point) const
{
	// Compute the crosspoint between this line and the line passing over the
	// point and the origin. If lines are parallel, point is not obstructed.
	std::pair<double, double> crossings = crosspoint(Line(point));
	if (std::isnan(crossings.first) || std::isnan(crossings.second))
		return false;

	// Otherwise, point is obstructed if the crosspoint
	// falls within the limits of both segments.
	return (
		0.0 < crossings.first && crossings.first < 1.0 &&
		0.0 < crossings.second && crossings.second < 1.0
	);
}

} // namespace virtual_scan

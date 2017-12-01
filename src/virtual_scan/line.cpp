#include "line.h"

#include <cmath>

namespace virtual_scan
{

Line::Line()
{
	// Nothing to do.
}


Line::Line(const carmen_position_t &point)
{
	static carmen_position_t origin = {0, 0};

	this->p1 = origin;
	this->p2 = point;
}


Line::Line(const carmen_position_t &p1, const carmen_position_t &p2)
{
	this->p1 = p1;
	this->p2 = p2;
}


carmen_position_t Line::operator () (double t) const
{
	carmen_position_t p = {p1.x + t * p2.x, p1.y + t * p2.y};
	return p;
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


bool Line::obstructs(const carmen_position_t &point) const
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

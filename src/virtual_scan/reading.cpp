#include "reading.h"

#include <limits>

namespace virtual_scan
{


Reading::Reading():
	timestamp(0),
	origin(0, 0, 0)
{
	// Nothing to do.
}


Reading::Reading(double timestamp, const Pose &origin):
	timestamp(timestamp),
	origin(origin)
{
	// Nothing to do.
}


Reading::Reading(carmen_mapper_virtual_scan_message *message):
	timestamp(message->timestamp)
{
	const carmen_point_t &globalpos = message->globalpos;
	origin.x = globalpos.x;
	origin.y = globalpos.y;
	origin.o = globalpos.theta;

	carmen_position_t *points = message->points;
	for (int i = 0, n = message->num_points; i < n; i++)
		emplace(origin.project_local(points[i]));
}


Reading::const_iterator Reading::lower_bound(double angle) const
{
	return std::set<Point2D, ComparatorOD>::lower_bound(Point2D(PointOD(angle, 0.0)));
}


Reading::const_iterator Reading::upper_bound(double angle) const
{
	static const double D_MAX = std::numeric_limits<double>::max();

	return std::set<Point2D, ComparatorOD>::upper_bound(Point2D(PointOD(angle, D_MAX)));
}


const_iterator_chain<Reading> Reading::lower_bound(const std::pair<double, double> &range) const
{
	// If the range lies between quadrants 2 and 3...
	if (range.second > M_PI)
	{
		// Select all points between the beginning of the range in quadrant 2 and pi radians.
		auto a_1 = lower_bound(range.first);
		auto b_1 = end();

		// Select all points between -pi radians and the end of the range in quadrant 3.
		auto a_2 = begin();
		auto b_2 = upper_bound(range.second - 2.0 * M_PI);

		// Return an iterator chaining both ranges.
		return const_iterator_chain<Reading>({a_1, b_1, a_2, b_2});
	}

	// Otherwise, return an iterator over a single range.
	return const_iterator_chain<Reading>({lower_bound(range.first), upper_bound(range.second)});
}


const_iterator_chain<Reading> Reading::upper_bound(const std::pair<double, double> &range) const
{
	// If the range lies between quadrants 2 and 3, return a
	// chained iterator pointing past the end of the reading;
	// otherwise, return an iterator past the range.
	auto n = (range.second <= M_PI ? upper_bound(range.second) : end());
	return const_iterator_chain<Reading>({n, n});
}


const Point2D &Reading::back() const
{
	return *rbegin();
}


const Point2D &Reading::front() const
{
	return *begin();
}


void Reading::merge(Reading &that)
{
	for (const Point2D point: that)
		insert(point);
}


} // namespace virtual_scan

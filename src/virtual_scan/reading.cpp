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

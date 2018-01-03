#include "segment.h"

#include "line.h"
#include "parameters.h"

#include <cmath>

namespace virtual_scan
{


Segment::Segment():
	Reading()
{
	// Nothing to do.
}


Segment::Segment(double timestamp, const Pose &origin):
	Reading(timestamp, origin)
{
	// Nothing to do.
}


Point2D Segment::centroid() const
{
	double x = 0.0;
	double y = 0.0;
	for (auto i = begin(), n = end(); i != n; ++i)
	{
		x += i->x;
		y += i->y;
	}

	x /= size();
	y /= size();

	return Point2D(PointXY(x, y));
}


// TODO: What happens if the segment lies on the border between quadrants 2 and 3 (i.e. around angle 180)?
inline void insert(double timestamp, const Pose &origin, const Point2D &point, Segment::S &segments)
{
	static const double D2_SEGMENT = D_SEGMENT * D_SEGMENT;

	// Iterate over segments from newest to oldest.
	for (auto i = segments.rbegin(), n = segments.rend(); i != n; ++i)
	{
		// Check the distance between the point and the tip of the given segment,
		// adding it if close enough.
		const Point2D &tip = i->back();
		if (distance2(point, tip) < D2_SEGMENT)
		{
			// Add point to segment (the iterator argument is a hint that the new
			// element is probably "higher" than any other already in the set).
			i->insert(i->end(), point);
			return;
		}
		
		// If the angle difference between point and tip is too large,
		// conclude there's no chance of finding a matching segment.
		if (std::abs(point.o - tip.o) > O_SEGMENT)
			break;
	}

	// If no existing segment matches the point, add it to a new one.
	segments.emplace_back(timestamp, origin);
	segments.back().insert(point);
}


inline void classify(Segment &segment)
{
	if (segment.size() < 3)
	{
		segment.shape = POINT_MASS;
		return;
	}

	const Point2D &a = segment.front();
	const Point2D &b = segment.back();
	if (distance2(a, b) < 0.5)
	{
		segment.shape = POINT_MASS;
		return;
	}

	Line line(a, b);

	Segment::iterator n = --(segment.end());
	Segment::iterator i = ++(segment.begin());
	Segment::iterator i_max = i;
	double d_max = 0.0;

	for (; i != n; ++i)
	{
		double d = line.distance(*i);
		if (d > d_max)
		{
			d_max = d;
			i_max = i;
		}
	}

	if (d_max < 0.5)
		segment.shape = I_SHAPED;
	else
	{
		segment.shape = L_SHAPED;
		segment.corner = *i_max;
	}
}


Segment::S split_segments(const Reading &reading)
{
	double timestamp = reading.timestamp;
	const Pose &origin = reading.origin;

	Segment::S segments;
	if (reading.size() == 0)
		return segments;

	for (auto i = reading.begin(), n = reading.end(); i != n; ++i)
		insert(timestamp, origin, *i, segments);

	for (auto i = segments.begin(), n = segments.end(); i != n; ++i)
		classify(*i);

	return segments;
}

} // namespace virtual_scan

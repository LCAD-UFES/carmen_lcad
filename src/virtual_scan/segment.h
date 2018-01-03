#ifndef VIRTUAL_SCAN_SEGMENT_H
#define VIRTUAL_SCAN_SEGMENT_H

#include "reading.h"

#include <vector>

namespace virtual_scan
{

/**
 * @brief Possible shapes of a segment.
 */
enum Shape
{
	POINT_MASS,
	I_SHAPED,
	L_SHAPED
};

/**
 * @brief A dense segment of sensor reading points.
 */
struct Segment: Reading
{
	/** @brief Segment sequence type. */
	typedef std::vector<Segment> S;

	/** @brief Shape of this segment. */
	Shape shape;

	/** @brief Corner point of this segment. Is only valid if `shape == L_SHAPED` is true. */
	Point2D corner;

	/**
	 * @brief Default constructor.
	 */
	Segment();

	/**
	 * @brief Create a new segment at the given time and origin pose.
	 */
	Segment(double timestamp, const Pose &origin);

	/**
	 * @brief Return the segment's centroid.
	 */
	Point2D centroid() const;
};

/**
 * @brief Split a sensor reading into a sequence of segments.
 */
Segment::S split_segments(const Reading &reading);

} // namespace virtual_scan

#endif

#ifndef VIRTUAL_SCAN_LINE_H
#define VIRTUAL_SCAN_LINE_H

#include "point.h"

#include <utility>

namespace virtual_scan
{

/**
 * @brief A 2D line represented in terms of a pair of points.
 */
class Line
{
	/** @brief First point in the segment defining the line. */
	PointXY p1;

	/** @brief Second point in the segment defining the line. */
	PointXY p2;

public:
	/**
	 * @brief Default constructor.
	 */
	Line();

	/**
	 * @brief Create a line passing over the origin and the given point.
	 */
	Line(const PointXY &point);

	/**
	 * @brief Create a new line described by the two given points.
	 */
	Line(const PointXY &p1, const PointXY &p2);

	/**
	 * @brief Compute the position distant `t` units from the line's first point, in the direction of the second point.
	 */
	PointXY operator () (double t) const;

	/**
	 * @brief Computes the parameters for the crosspoint between two lines.
	 *
	 * The `first` parameter is for `this` line, and the `second` for `that` line.
	 *
	 * See: https://stackoverflow.com/a/4977569/476920
	 */
	std::pair<double, double> crosspoint(const Line &that) const;

	/**
	 * @brief Return the distance between the line and the given point.
	 */
	double distance(const PointXY &point) const;

	/**
	 * @brief Computes whether this line obstructs the view of the given point from the origin.
	 */
	bool obstructs(const PointXY &point) const;
};

} // namespace virtual_scan

#endif

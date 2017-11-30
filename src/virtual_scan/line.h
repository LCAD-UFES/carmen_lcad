#ifndef VIRTUAL_SCAN_LINE_H
#define VIRTUAL_SCAN_LINE_H

#include <carmen/carmen.h>

#include <utility>

namespace virtual_scan
{

/**
 * @brief A 2D line represented in terms of a pair of points.
 */
class Line
{
	/** @brief First point in the segment defining the line. */
	carmen_position_t p1;

	/** @brief Second point in the segment defining the line. */
	carmen_position_t p2;

public:
	/**
	 * @brief Default constructor.
	 */
	Line();

	/**
	 * @brief Create a line passing over the origin and the given point.
	 */
	Line(const carmen_position_t &point);

	/**
	 * @brief Create a new line described by the two given points.
	 */
	Line(const carmen_position_t &p1, const carmen_position_t &p2);

	/**
	 * @brief Compute the position distant `t` units from the line's first point, in the direction of the second point.
	 */
	carmen_position_t operator () (double t) const;

	/**
	 * @brief Computes the parameters for the crosspoint between two lines.
	 *
	 * The `first` parameter is for `this` line, and the `second` for `that` line.
	 *
	 * See: https://stackoverflow.com/a/4977569/476920
	 */
	std::pair<double, double> crosspoint(const Line &that) const;

	/**
	 * @brief Computes whether this line obstructs the view of the given point from the origin.
	 */
	bool obstructs(const carmen_position_t &point) const;
};

} // namespace virtual_scan

#endif

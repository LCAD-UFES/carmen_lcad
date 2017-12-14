#ifndef VIRTUAL_SCAN_READING_H
#define VIRTUAL_SCAN_READING_H

#include "point.h"

#include <set>

namespace virtual_scan
{

struct Reading: std::set<Point2D, ComparatorOD>
{
	/** @brief Time when this reading was collected. */
	double timestamp;

	/** @brief Observer pose at the time this reading was collected. */
	Pose origin;

	/**
	 * @brief Default constructor.
	 */
	Reading();

	/**
	 * @brief Create a new empty reading at the given time and origin pose.
	 */
	Reading(double timestamp, const Pose &origin);

	/**
	 * @brief Create a new reading from the given message.
	 */
	Reading(carmen_mapper_virtual_scan_message *message);

	/**
	 * @brief Return an iterator placed on the first point at the given angle.
	 */
	const_iterator lower_bound(double angle) const;

	/**
	 * @brief Return an iterator placed on the first point at the given angle.
	 */
	const_iterator upper_bound(double angle) const;

	/**
	 * @brief Return the last point in this reading.
	 */
	const Point2D &back() const;

	/**
	 * @brief Return the first point in this reading.
	 */
	const Point2D &front() const;
};

} // namespace virtual_scan

#endif

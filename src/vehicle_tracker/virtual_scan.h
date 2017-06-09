#ifndef VIRTUALSCAN_H
#define VIRTUALSCAN_H

#include "types.h"

#include <carmen/carmen.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/virtual_scan_messages.h>

#include <vector>

struct VirtualScan
{
	/** @brief Center of this virtual scan in the global reference frame. */
	g2d::Point center;

	/** @brief Orientation angle of this virtual scan in the global reference frame. */
	g2d::Field angle;

	/** @brief Cartesian representation of virtual scan rays relative to the center coordinates. */
	std::vector<g2d::Point> cartesian;

	/** @brief Radial representation of virtual scan rays relative to the center coordinates. */
	std::vector<g2d::Point> polar;

	/**
	 * @brief Deafult constructor.
	 */
	VirtualScan();

	/**
	 * @brief Create a new virtual scan object from a set of CARMEM messages.
	 *
	 * @param globalpos Global position message.
	 *
	 * @param virtual_scan Virtual scan message.
	 */
	VirtualScan(carmen_localize_ackerman_globalpos_message *globalpos, carmen_virtual_scan_message *virtual_scan);

	/**
	 * @brief Compute a difference scan as the set of points in `b` not found in `a`, with center displacement taken in account.
	 */
	VirtualScan(const VirtualScan &a, const VirtualScan &b);

	/**
	 * @brief Add the given cartesian point to this virtual scan.
	 */
	void append(const g2d::Point &p);

	/**
	 * @brief Add the given polar point to this virtual scan.
	 */
	void append(g2d::Field t, g2d::Field d);

	/**
	 * @brief Return the number of points in the scan.
	 */
	size_t size() const;
};

#endif

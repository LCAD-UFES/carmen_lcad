#ifndef OFFLINE_MAP_H
#define OFFLINE_MAP_H

#include "types.h"

#include <carmen/carmen.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/map_server_messages.h>
#include <carmen/virtual_scan_messages.h>

#include <vector>

struct OfflineMap
{
	/** @brief Center of this offline map in the global reference frame. */
	g2d::Point center;

	/** @brief Resolution of the offline map. */
	g2d::Field resolution;

	/** @brief Cartesian representation of map points relative to the center coordinates. */
	struct
	{
		// Sequence containing all map points.
		std::vector<g2d::Point> points;

		// Clusters of points.
		std::vector<g2d::Points> clusters;
	}
	cartesian;


	/** @brief Radial representation of map points relative to the center coordinates. */
	std::vector<g2d::Point> polar;

	double range;

	/**
	 * @brief Deafult constructor.
	 */
	OfflineMap();

	/**
	 * @brief Create a new offline map object from a set of CARMEM messages.
	 *
	 * @param range Range of offline maps displayed.
	 *
	 * @param globalpos Global position message.
	 *
	 * @param map_message Virtual scan message.
	 */
	OfflineMap(double range, carmen_localize_ackerman_globalpos_message *globalpos, carmen_mapper_map_message *map_message);

	/**
	 * @brief Return the number of points in the scan.
	 */
	size_t size() const;

private:
	/** @brief Grid representation of the offline map. */
};

#endif

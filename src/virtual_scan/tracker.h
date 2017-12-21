/*
 * tracker.h
 *
 *  Created on: Nov 14, 2017
 *	  Author: claudine
 */

#ifndef VIRTUAL_SCAN_TRACKER_H
#define VIRTUAL_SCAN_TRACKER_H

#include "neighborhood_graph.h"
#include "random.h"
#include "tracks.h"

namespace virtual_scan
{

/**
 * @brief Moving obstacle tracker.
 */
class Tracker
{
	/** @brief Sequence of sensor readings over the time window. */
	Readings readings;

	/** @brief Neighborhood graph used to generate track hypotheses. */
	Graph graph;

	/** @brief Pointer to current tracks. */
	Tracks::P tracks;

	/** @brief Random number generator used to select track variations. */
	std::uniform_real_distribution<> uniform;

public:
	/**
	 * @brief Default constructor.
	 */
	Tracker();

	/**
	 * @brief Update the current tracking hypothesis with the given data.
	 *
	 * Return a pointer to the updated tracking hypothesis.
	 */
	Tracks::P track(carmen_mapper_virtual_scan_message *message);
};

} // namespace virtual_scan

#endif // VIRTUAL_SCAN_TRACKER_H

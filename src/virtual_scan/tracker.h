/*
 * tracker.h
 *
 *  Created on: Nov 14, 2017
 *	  Author: claudine
 */

#ifndef VIRTUAL_SCAN_TRACKER_H
#define VIRTUAL_SCAN_TRACKER_H

#include "random.h"
#include "tracks.h"
#include "virtual_scan.h"
#include "virtual_scan_neighborhood_graph.h"

namespace virtual_scan
{

class Tracker
{
	int n_mc;
	virtual_scan_neighborhood_graph_t *neighborhood_graph;

	Readings readings;

	/** @brief Pointer to current tracks. */
	Tracks::P tracks;

	std::uniform_real_distribution<> uniform;

	bool keep(Tracks *new_tracks);

public:
	Tracker(int n, int T);

	Tracks::P track(
		virtual_scan_box_model_hypotheses_t *box_model_hypotheses,
		virtual_scan_extended_t *virtual_scan_extended
	);
};

} // namespace virtual_scan

#endif // VIRTUAL_SCAN_TRACKER_H

/*
 * tracker.h
 *
 *  Created on: Nov 14, 2017
 *      Author: claudine
 */

#ifndef SRC_VIRTUAL_SCAN_TRACKER_H_
#define SRC_VIRTUAL_SCAN_TRACKER_H_

#include <vector>
#include <random>
#include "virtual_scan_neighborhood_graph.h"

namespace virtual_scan
{

class Track
{
	std::vector<virtual_scan_box_model_t> box_model;
};

class Tracks
{
	std::vector<Track> tracks;
	std::random_device *rd;

	Tracks *
	track_extension(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	Tracks *
	track_reduction(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	Tracks *
	track_birth(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	Tracks *
	track_death(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	Tracks *
	track_split(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	Tracks *
	track_merge(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	Tracks *
	track_switch(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	Tracks *
	track_diffusion(virtual_scan_neighborhood_graph_t *neighborhood_graph);
public:
	Tracks(std::random_device *rd);
	Tracks *
	propose(neighborhood_graph_t *neighborhood_graph);
	double
	P();
};

class Tracker
{
	int n_mc;
	virtual_scan_neighborhood_graph_t *neighborhood_graph;
	Tracks *tracks_n;
	Tracks *tracks_star;

	std::random_device rd;
	std::uniform_real_distribution<> uniform;

	bool keep(Tracks *new_tracks);

public:
	Tracker(int n);
	Tracks *track(virtual_scan_box_model_hypotheses_t *box_model_hypotheses);
};

} /* namespace virtual_scan */


#endif /* SRC_VIRTUAL_SCAN_TRACKER_H_ */




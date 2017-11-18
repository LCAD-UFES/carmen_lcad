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
	std::deque<virtual_scan_graph_node_t *> graph_nodes;

public:
	void append_front(virtual_scan_graph_node_t *node);
	void append_back(virtual_scan_graph_node_t *node);
	virtual_scan_graph_node_t *Track::front_node();
	virtual_scan_graph_node_t *Track::back_node();
};

class Tracks
{
	std::vector<Track> tracks;
	std::random_device *rd;

	void track_extension(virtual_scan_neighborhood_graph_t *neighborhood_graph, Track *tau);
	void track_extension(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	void track_reduction(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	void track_birth(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	void track_death(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	void track_split(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	void track_merge(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	void track_switch(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	void track_diffusion(virtual_scan_neighborhood_graph_t *neighborhood_graph);
public:
	Tracks(std::random_device *rd);
	Tracks *propose(neighborhood_graph_t *neighborhood_graph);
	double P();
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




/*
 * tracker.h
 *
 *  Created on: Nov 14, 2017
 *      Author: claudine
 */

#ifndef SRC_VIRTUAL_SCAN_TRACKER_H_
#define SRC_VIRTUAL_SCAN_TRACKER_H_

#include <random>

namespace virtual_scan
{

class Track
{
	vector<virtual_scan_box_model_t> box_model;
};

class Tracks
{
	vector<Track> tracks;
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




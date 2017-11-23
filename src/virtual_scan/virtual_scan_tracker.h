/*
 * tracker.h
 *
 *  Created on: Nov 14, 2017
 *      Author: claudine
 */

#ifndef SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_TRACKER_H_
#define SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_TRACKER_H_

#include <carmen/carmen.h>

#include "virtual_scan.h"
#include "virtual_scan_neighborhood_graph.h"

#include <deque>
#include <random>
#include <vector>

namespace virtual_scan
{

class Obstacle
{
public:
	virtual_scan_graph_node_t *graph_node;
	double x;
	double y;
	double theta;
	Obstacle();
	Obstacle(virtual_scan_graph_node_t *graph_node);
	virtual_scan_graph_node_t *operator -> (); // Operator overload
};

class Track
{
	std::deque<Obstacle> graph_nodes;

public:
	~Track();
	int size();
	void append_front(virtual_scan_graph_node_t *node);
	void append_back(virtual_scan_graph_node_t *node);
	virtual_scan_graph_node_t *front_node();
	virtual_scan_graph_node_t *back_node();
	void track_forward_reduction(int r);
	void track_backward_reduction(int r);
	void track_move(Track *track, int s);
	bool is_mergeable(Track *tau);
	void track_merge(Track *track);
	std::pair <int, int> is_switchable(Track *tau);
	void track_switch(Track *tau, std::pair <int, int> break_point_pair);
	void track_update(std::random_device *rd);
	double P_L(double lambda_L);
};

class Tracks
{
	std::vector<Track> tracks;
	std::random_device *rd;

	bool track_extension(Track *tau);
	bool track_extension();
	bool forward_extension(Track *tau);
	bool backward_extension(Track *tau);
	bool track_reduction();
	bool track_birth(virtual_scan_neighborhood_graph_t *neighborhood_graph);
	bool track_death();
	bool track_split();
	bool track_merge();
	bool track_switch();
	bool track_diffusion();
public:
	Tracks(std::random_device *rd);
	Tracks *propose(virtual_scan_neighborhood_graph_t *neighborhood_graph);
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
	Tracks *track(virtual_scan_box_model_hypotheses_t *box_model_hypotheses,
			virtual_scan_extended_t *virtual_scan_extended);
};

} /* namespace virtual_scan */


#endif /* SRC_VIRTUAL_SCAN_VIRTUAL_SCAN_TRACKER_H_ */




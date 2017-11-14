#include "tracker.h"

namespace virtual_scan
{

Tracker::Tracker(int n):
	n_mc(n),
	neighborhood_graph(virtual_scan_alloc_neighborhood_graph ()),
	tracks_n(new Tracks()),
	tracks_star(new Tracks()),
	uniform(0.0, 1.0),
	
{
}

bool Tracker::keep(Tracks *new_tracks)
{
	double U = uniform(rd);
	
}

Tracks *Tracker::track(virtual_scan_box_model_hypotheses_t *box_model_hypotheses)
{
	update_neighborhood_graph(neighborhood_graph, box_model_hypotheses);
	for (int n = 0; n < n_mc; n++)
	{
		Tracks *new_tracks = tracks_n->propose(neighborhood_graph);
		
		if (keep(new_tracks)) 
		{
			if (tracks_n != tracks_star)
				delete tracks_n;
			tracks_n = new_tracks;
			if (prob(tracks_n) > prob (tracks_star))
			{
				delete tracks_star;
				tracks_star = tracks_n;
			}		
		}
		else 
			delete new_tracks;	
	}
	
	return tracks_star;
}

}


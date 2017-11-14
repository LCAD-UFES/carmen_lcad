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


double Tracks::P()
{
	
}


bool Tracker::keep(Tracks *tracks_new)
{
	double U = uniform(rd);
	return (U < (tracks_new->P() / tracks_n->P()));
}

Tracks *Tracker::track(virtual_scan_box_model_hypotheses_t *box_model_hypotheses)
{
	update_neighborhood_graph(neighborhood_graph, box_model_hypotheses);
	for (int n = 0; n < n_mc; n++)
	{
		Tracks *tracks_new = tracks_n->propose(neighborhood_graph);
		
		if (keep(tracks_new)) 
		{
			if (tracks_n != tracks_star)
				delete tracks_n;
			tracks_n = tracks_new;
			if (prob(tracks_n) > prob (tracks_star))
			{
				delete tracks_star;
				tracks_star = tracks_n;
			}		
		}
		else 
			delete tracks_new;	
	}
	
	return tracks_star;
}

}


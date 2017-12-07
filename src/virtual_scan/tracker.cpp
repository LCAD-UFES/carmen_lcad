#include "tracker.h"

namespace virtual_scan
{


Tracker::Tracker(int n, int T):
	n_mc(n),
	neighborhood_graph(virtual_scan_initiate_neighborhood_graph(T)),
	tracks(new Tracks()),
	uniform(0.0, 1.0)
{
}


// MCMC Sampler
Tracks::P Tracker::track(virtual_scan_box_model_hypotheses_t *box_model_hypotheses, virtual_scan_extended_t *virtual_scan_extended)
{
	Tracks::P tracks_n = tracks;
	update_neighborhood_graph(neighborhood_graph, box_model_hypotheses, virtual_scan_extended);
	for (int n = 0; n < n_mc; n++)
	{
		Tracks::P tracks_new = tracks_n->propose(neighborhood_graph);
		double U = uniform(RD);
		if (U < (tracks_new->P_wZ() / tracks_n->P_wZ())) 
		{
			tracks_n = tracks_new;
			if (tracks_n->P_wZ() > tracks->P_wZ())
				tracks = tracks_n;
		}
	}
	
	return tracks;
}


} // namespace virtual_scan

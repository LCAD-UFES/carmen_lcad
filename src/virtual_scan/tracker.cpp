#include "tracker.h"

namespace virtual_scan
{

Tracker::Tracker(int n):
	n_mc(n),
	neighborhood_graph(virtual_scan_alloc_neighborhood_graph ()),
	tracks_n(new Tracks(&rd)),
	tracks_star(new Tracks(&rd)),
	uniform(0.0, 1.0)
{
}

Tracks::Tracks(std::random_device *rd_):
	rd(rd_)
{
}

Tracks *Tracks::track_birth(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{
	std::uniform_int_distribution<> uniform(0, neighborhood_graph->num_sub_graphs);
	int n = uniform(*rd);

	virtual_scan_disconnected_sub_graph_t *disconnected_sub_graph = neighborhood_graph->first;
	for (int i = 0; i < n; i++)
		disconnected_sub_graph = disconnected_sub_graph->next;

	std::uniform_int_distribution<> uniform(0, neighborhood_graph->first);
}

Tracks *track_extension(virtual_scan_neighborhood_graph_t *neighborhood_graph)
{

}

Tracks *Tracks::propose(neighborhood_graph_t *neighborhood_graph)
{

}

double Tracks::P()
{
	
}

Tracks *Tracker::track(virtual_scan_box_model_hypotheses_t *box_model_hypotheses)
{
	update_neighborhood_graph(neighborhood_graph, box_model_hypotheses);
	for (int n = 0; n < n_mc; n++)
	{
		Tracks *tracks_new = tracks_n->propose(neighborhood_graph);
		double U = uniform(rd);
		if (U < (tracks_new->P() / tracks_n->P())) 
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


#include "tracker.h"

#include "parameters.h"

namespace virtual_scan
{


Tracker::Tracker():
	tracks(new Tracks()),
	uniform(0.0, 1.0)
{
	// Nothing to do.
}


// MCMC Sampler
Tracks::P Tracker::track(carmen_mapper_virtual_scan_message *message)
{
	readings.update(message);
	graph.update(readings.back());
	tracks->update(readings);

	Tracks::P tracks_n = tracks;
	for (size_t n = 0; n < N_MC; n++)
	{
		Tracks::P tracks_new = tracks_n->propose(graph);
		double U = uniform(RD);
		if (U < (tracks_new->PwZ() / tracks_n->PwZ())) 
		{
			tracks_n = tracks_new;
			if (tracks_n->PwZ() > tracks->PwZ())
				tracks = tracks_n;
		}
	}
	
	return tracks;
}


} // namespace virtual_scan

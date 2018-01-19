#include "tracker.h"

//#undef DEBUG
#include "logging.h"

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
ObstaclePose::S Tracker::track(carmen_mapper_virtual_scan_message *message)
{
	readings.update(message);
	tracks->update(readings);
	graph.update(readings.back());

	Tracks::P tracks_n = tracks;
	for (size_t n = 0; n < N_MC; n++)
	{
		// Try to create a variation of the current Tracls object,
		// giving up on the MCMC operation if it can't be created.
		Tracks::P tracks_new = tracks_n->propose(graph);
		if (tracks_new.use_count() == 0)
			break;

		double U = uniform(RD);
		if (U < (tracks_new->PwZ() / tracks_n->PwZ())) 
		{
			tracks_n = tracks_new;
			if (tracks_n->PwZ() > tracks->PwZ())
				tracks = tracks_n;
		}
	}
	
	ObstaclePose::S obstacles = tracks->obstacles();

	LOG("Tracking obstacles: " << obstacles.size());

	return obstacles;
}


} // namespace virtual_scan

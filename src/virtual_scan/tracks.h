#ifndef VIRTUAL_SCAN_TRACKS_H
#define VIRTUAL_SCAN_TRACKS_H

#include "posterior.h"
#include "track.h"

#include <memory>

namespace virtual_scan
{

/**
 * @brief A sequence of tracks along with Markov dynamics and posterior probability.
 */
class Tracks
{
	/** @brief Track objects contained in this sequence. */
	Track::S tracks;

	/**
	 * @brief Create a new track.
	 */
	bool create(virtual_scan_neighborhood_graph_t *neighborhood_graph);

	/**
	 * @brief Erase an existing track.
	 */
	bool destroy();

	/**
	 * @brief Randomly select a track and an extension method (forward or backward), then apply it to the selected track.
	 */
	bool extend();

	/**
	 * @brief Randomly select an extension method (forward or backward), then apply it to the given track.
	 */
	bool extend(Track &tau);

	/**
	 * @brief Extend the given track forward.
	 */
	bool extend_forward(Track &tau);

	/**
	 * @brief Extend the given track backward.
	 */
	bool extend_backward(Track &tau);

	/**
	 * @brief Reduce a randomly selected track.
	 */
	bool reduce();

	/**
	 * @brief Split an existing track in two.
	 */
	bool split();

	/**
	 * @brief Merge two tracks in one.
	 */
	bool merge();

	/**
	 * @brief Swap parts of two tracks between them.
	 */
	bool swap();

	/**
	 * @brief Change a randomly selected track pose.
	 */
	bool diffuse();

	/**
	 * @brief Compute the point matching probability for this track set.
	 */
	double P_M1(int i, virtual_scan_neighborhood_graph_t *neighborhood_graph) const;

public:
	/** @brief Shared Tracks pointer type. */
	typedef std::shared_ptr<Tracks> P;

	/** @brief Posterior probability of this track sequence. */
	Posterior P_wZ;

	/**
	 * @brief Return the `index`th Track object in this sequence.
	 */
	//Track &operator[] (size_t index);

	/**
	 * @brief Return the `index`th Track object in this sequence.
	 */
	//const Track &operator[] (size_t index) const;

	Tracks::P propose(virtual_scan_neighborhood_graph_t *neighborhood_graph);
};

} // namespace virtual_scan

#endif

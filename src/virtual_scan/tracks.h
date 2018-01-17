#ifndef VIRTUAL_SCAN_TRACKS_H
#define VIRTUAL_SCAN_TRACKS_H

#include "neighborhood_graph.h"
#include "posterior.h"
#include "readings.h"
#include "track.h"

#include <memory>

namespace virtual_scan
{

/**
 * @brief A sequence of tracks along with Markov dynamics and posterior probability.
 */
struct Tracks: Track::S
{
	/** @brief Shared Tracks pointer type. */
	typedef std::shared_ptr<Tracks> P;

	/** @brief Posterior probability of this track sequence. */
	Posterior PwZ;

	/**
	 * @brief Return a sequence containing the last pose of each track in this collection.
	 */
	ObstaclePose::S obstacles() const;

	/**
	 * @brief Returns a random variation of this track collection, generated from the given neighborhood graph.
	 */
	Tracks::P propose(Graph &graph);

	/**
	 * @brief Update tracks and the posterior distribution to reflect changes in sensor information.
	 */
	void update(const Readings &readings);

private:
	/**
	 * @brief Create a new track.
	 */
	bool create(Graph &graph);

	/**
	 * @brief Erase an existing track.
	 */
	bool destroy();

	/**
	 * @brief Erase the track of given index.
	 */
	bool destroy(size_t n);

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
};

/**
 * @brief Print a Tracks object to to an output stream.
 */
std::ostream &operator << (std::ostream &out, const Tracks &tracks);

} // namespace virtual_scan

#endif

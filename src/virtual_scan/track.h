#ifndef VIRTUAL_SCAN_TRACK
#define VIRTUAL_SCAN_TRACK

#include "obstacle.h"

#include <carmen/carmen.h>

#include <deque>

namespace virtual_scan
{

/**
 * @brief A sequence of obstacle configurations over time.
 */
struct Track: ObstaclePose::S
{
	/** @brief Unique Track ID type. */
	typedef Track* ID;

	/** @brief Alias for a sequence of tracks. */
	typedef std::deque<Track> S;

	/** @brief Unique ID associated to this track. */
	ID id;

	/**
	 * @brief Default constructor.
	 */
	Track();
	
	/**
	 * @brief Remove all poses from `(r + 1)` to the end of the track.
	 */
	void pop_back(int r);

	/**
	 * @brief Move all poses from `(r + 1)` to the end of `this` track to `that` track.
	 */
	void pop_back(int r, Track &that);

	/**
	 * @brief Remove all poses from the beginning of the track up to `(r - 1)`.
	 */
	void pop_front(int r);
	
	bool is_mergeable(const Track &that) const;
	
	void merge(Track &that);
	
	/**
	 * @brief Update a random pose in the track.
	 */
	int diffuse();
};

} // namespace virtual_scan

#endif

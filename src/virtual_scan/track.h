#ifndef VIRTUAL_SCAN_TRACK
#define VIRTUAL_SCAN_TRACK

#include "obstacle.h"

#include <carmen/carmen.h>

#include <memory>
#include <vector>

namespace virtual_scan
{

/**
 * @brief A sequence of obstacle configurations over time.
 */
class Track
{
	/** @brief Sequence of obstacle configurations. */
	ObstaclePose::S poses;

public:
	/** @brief Unique Track ID type. */
	typedef Track* ID;

	/** @brief Reference-counted Track pointer type. */
	typedef std::shared_ptr<Track> P;

	/** @brief Alias for a vector of tracks. */
	typedef std::vector<Track::P> S;

	/** @brief Unique ID associated to this track. */
	ID id;

	/**
	 * @brief Default constructor.
	 */
	Track();

	/**
	 * @brief Return the obstace pose at the given position in the Track.
	 */
	ObstaclePose &operator[] (int index);

	/**
	 * @brief Return the obstace pose at the given position in the Track.
	 */
	const ObstaclePose &operator[] (int index) const;

	/**
	 * @brief Return a reference to the last pose in this track.
	 */
	ObstaclePose &back();

	/**
	 * @brief Return a reference to the last pose in this track.
	 */
	const ObstaclePose &back() const;

	/**
	 * @brief Add a new obstacle pose to the beginning of this track, created from the given neighborhood graph node.
	 */
	void push_front(Node *node);

	/**
	 * @brief Add a new obstacle pose to the end of this track, created from the given neighborhood graph node.
	 */
	void push_back(Node *node);

	/**
	 * @brief Return the neighborhood graph node associated to the obstacle pose at the given index.
	 */
	Node *at_node(int index);

	/**
	 * @brief Return the neighborhood graph node associated to the obstacle pose at the given index.
	 */
	const Node *at_node(int index) const;

	/**
	 * @brief Return the neighborhood graph node associated to the first obstacle pose.
	 */
	Node *front_node();

	/**
	 * @brief Return the neighborhood graph node associated to the first obstacle pose.
	 */
	const Node *front_node() const;

	/**
	 * @brief Return the neighborhood graph node associated to the last obstacle pose.
	 */
	Node *back_node();
	
	/**
	 * @brief Return the neighborhood graph node associated to the last obstacle pose.
	 */
	const Node *back_node() const;
	
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
	
	/**
	 * @brief Return the length of this track, in number of configurations.
	 */
	size_t size() const;
};

} // namespace virtual_scan

#endif

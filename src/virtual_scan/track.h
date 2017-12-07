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
class Track
{
	/** @brief Sequence of obstacle configurations. */
	std::deque<ObstaclePose> poses;

public:
	/** @brief Alias for a vector of tracks. */
	typedef std::vector<Track> S;

	/**
	 * @brief Class destructor.
	 */
	~Track();

	/**
	 * @brief Add a new obstacle pose to the beginning of this track, created from the given neighborhood graph node.
	 */
	void push_front(virtual_scan_graph_node_t *node);

	/**
	 * @brief Add a new obstacle pose to the end of this track, created from the given neighborhood graph node.
	 */
	void push_back(virtual_scan_graph_node_t *node);

	/**
	 * @brief Return the neighborhood graph node associated to the obstacle pose at the given index.
	 */
	virtual_scan_graph_node_t *at_node(int index);

	/**
	 * @brief Return the neighborhood graph node associated to the first obstacle pose.
	 */
	virtual_scan_graph_node_t *front_node();

	/**
	 * @brief Return the neighborhood graph node associated to the first obstacle pose.
	 */
	const virtual_scan_graph_node_t *front_node() const;

	/**
	 * @brief Return the neighborhood graph node associated to the last obstacle pose.
	 */
	virtual_scan_graph_node_t *back_node();
	
	/**
	 * @brief Return the neighborhood graph node associated to the last obstacle pose.
	 */
	const virtual_scan_graph_node_t *back_node() const;
	
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
	
    double P_L(double lambda_L, int T);

	/**
	 * @brief Compute the temporal consistency probability of this track.
	 */
	double P_T() const;


	/**
	 * @brief Add the view(s) of this track at time `t` from global pose `globalpos` to the given sequence.
	 */
	void push_view(int t, const carmen_point_t &globalpos, std::vector<ObstacleView> &w) const;

	/**
	 * @brief Return the length of this track, in number of configurations.
	 */
	size_t size() const;
};

} // namespace virtual_scan

#endif

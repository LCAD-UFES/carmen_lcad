#ifndef VIRTUAL_SCAN_POSTERIOR_H
#define VIRTUAL_SCAN_POSTERIOR_H

#include "readings.h"
#include "track.h"

#include <map>

namespace virtual_scan
{

/**
 * @brief Class used to manage exchanges between dynamic and static readings.
 */
class Collector
{
	/** @brief Whether to collect hitting or missing points. */
	bool hits;

	/** @brief Sequence of collected points. */
	std::vector<Point2D> points;

public:
	/**
	 * Create a new collector of given mode.
	 */
	Collector(bool hits);

	/**
	 * @brief Collect the given point if its mode agrees with the object's.
	 */
	void operator() (bool mode, const Point2D &point);

	/**
	 * @brief Move points from origin to destination reading.
	 */
	void move(Reading &Z_i, Reading &Z_j);
};

/**
 * @brief Algorithm class for computing the posterior probability of a sequence of Track objects.
 */
class Posterior
{
	/** @brief Map of track lengths. */
	std::map<Track::ID, int> lengths;

	/** @brief Map of track covariances. */
	std::map<Track::ID, double> covariances;

	/** @brief Sums of ray distances, indexed by generating obstacle. */
	std::map<const Node*, double> distances;

	/** @brief Dynamic sensor readings, indexed by generating obstacle. */
	std::map<const Node*, Reading> Zd;

	/** @brief Static sensor readings. */
	Readings Zs;

	/** @brief Sum of the lengths of all tracks. */
	double S_len;

	/** @brief Sum of the covariances of all tracks. */
	double S_mot;

	/** @brief Sum of the distances between dynamic sensor readings and their generating obstacles. */
	double S_ms1;

	/** @brief Number of static sensor readings found in areas occupied by obstacles in some earlier or later time. */
	double S_ms3;

	/**
	 * @brief Erase entries from the `distances` and `Zd` maps, updating the values of `S_ms1` and `S_ms3` accordingly.
	 *
	 * @param keep_sensor_points Whether sensor points associated to the to-be-erased node should be reassigned as static or discarded.
	 *
	 * @param i Index of the track containing the node to erase (through their enclosing obstacle poses).
	 *
	 * param j Index of the pose containing the node to erase.
	 *
	 * @param tracks Sequence containing the target track.
	 */
	void erase_node(bool keep_sensor_points, int i, int j, const Track::S &tracks);

	/**
	 * @brief Update the value of the `S_len` parameter with the given track's new length.
	 *
	 * @param tracks Reference to the updated track.
	 */
	void update_S_len(const Track &track);

	/**
	 * @brief Update the value of the `S_len` parameter with the given track's new length.
	 *
	 * @param i Position of the track in the sequence.
	 *
	 * @param tracks Sequence containing the updated track.
	 */
	void update_S_len(int i, const Track::S &tracks);

	/**
	 * @brief Update the value of the `S_mot` parameter with the given track's new covariance.
	 *
	 * @param track Reference to the updated track.
	 */
	void update_S_mot(const Track &track);

	/**
	 * @brief Update the value of the `S_mot` parameter with the given track's new covariance.
	 *
	 * @param i Position of the track in the sequence.
	 *
	 * @param tracks Sequence containing the updated track.
	 */
	void update_S_mot(int i, const Track::S &tracks);

	/**
	 * @brief Update the value of the `S_mot` parameter with the recomputed covariance for a to-be-shortened track.
	 *
	 * @param a Index of the first retained track pose.
	 *
	 * @param n Index just past the last retained track pose.
	 *
	 * @param track Reference to the to-be-shortened track.
	 */
	void update_S_mot(int a, int n, const Track &track);

	/**
	 * @brief Update the distances between dynamic sensor readings and a given obstacle pose.
	 *
	 * This method also updates the internal dynamic and static sensor reading
	 * collections used by other update methods.
	 *
	 * @param j Position of the track pose to update.
	 *
	 * @param track Reference to the updated track.
	 */
	void update_S_ms1(int j, const Track &track);

	/**
	 * @brief Update the distances between dynamic sensor readings and a given obstacle pose.
	 *
	 * This method also updates the internal dynamic and static sensor reading
	 * collections used by other update methods.
	 *
	 * @param i Position of the given track in the sequence.
	 *
	 * @param j Position of the track pose to update.
	 *
	 * @param tracks Sequence containing the updated track.
	 */
	void update_S_ms1(int i, int j, const Track::S &tracks);

	/**
	 * @brief Update the distances between the given object and reading rays.
	 *
	 * If `ranged` is `true`, only rays that intercept the obstacle are processed.
	 *
	 * Rays that do or don't hit the object at the given pose will be collected for
	 * reassignment according to the configuration of the given Collector object.
	 */
	void update_S_ms1(const ObstaclePose &pose, const Reading &Z_k, bool ranged, Collector &collect);

	/**
	 * @brief Update the counter for static sensor readings observed in areas occupied by an obstacle at some point in time.
	 *
	 * @param j Position of the track pose to update.
	 *
	 * @param track Reference to the updated track.
	 */
	void update_S_ms3(int j, const Track &track);

	/**
	 * @brief Update the counter for static sensor readings observed in areas occupied by an obstacle at some point in time.
	 *
	 * @param i Position of the given track in the sequence.
	 *
	 * @param j Position of the track pose to update.
	 *
	 * @param tracks Sequence containing the updated track.
	 */
	void update_S_ms3(int i, int j, const Track::S &tracks);

	/**
	 * @brief Update the counter for static sensor readings observed in areas occupied by an obstacle at some point in time.
	 * 
	 * @param tracks Sequence of tracks containing the obstacle poses to check.
	 *
	 * @param Z Collection of sensor readings to check.
	 */
	void update_S_ms3(const Track::S &tracks, const Reading &Z);
public:
	/**
	 * @brief Default constructor.
	 */
	Posterior();

	/**
	 * @brief Return the current posterior value.
	 */
	double operator () () const;

	/**
	 * @brief Update the posterior is response to the creation of a new track.
	 *
	 * @param i Index of the track in its parent sequence.
	 *
	 * @param tracks Sequence containing the created track.
	 */
	void create(int i, const Track::S &tracks);

	/**
	 * @brief Update the posterior is response to the destruction of a track.
	 *
	 * This method is called just prior to the actual destruction of the track.
	 *
	 * @param i Index of the track in its parent sequence.
	 *
	 * @param tracks Sequence containing the track to be destroyed.
	 */
	void destroy(int i, const Track::S &tracks);

	/**
	 * @brief Update the posterior in response to a track being extended forward.
	 */
	void extend_forward(const Track &track);

	/**
	 * @brief Update the posterior in response to a track being extended backward.
	 */
	void extend_backward(const Track &track);

	/**
	 * @brief Update the posterior in responde to removing all later poses from index `k` onwards.
	 *
	 * This method is called just prior to the actual shortening of the track.
	 *
	 * @param i Index of the track in its parent sequence.
	 *
	 * @param k Index of the first pose to be removed.
	 *
	 * @param tracks Sequence containing the track to be shortened.
	 *
	 */
	void pop_back(int i, int k, const Track::S &tracks);

	/**
	 * @brief Update the posterior in responde to removing the first `k` poses.
	 *
	 * This method is called just prior to the actual shortening of the track.
	 *
	 * @param i Index of the track in its parent sequence.
	 *
	 * @param k Index just past the last pose to be removed.
	 *
	 * @param tracks Sequence containing the track to be shortened.
	 *
	 */
	void pop_front(int i, int k, const Track::S &tracks);

	/**
	 * @brief Update the posterior in response to splitting a track in two.
	 */
	void split(const Track &track_1, const Track &track_2);

	/**
	 * @brief Update the posterior in response to merging two tracks.
	 *
	 * This method is called just after the tracks are merged, so `track_2` will be empty.
	 */
	void merge(const Track &track_1, const Track &track_2);

	/**
	 * @brief Update the posterior is response to the shortening of a track.
	 *
	 * This method is called just prior to the actual shortening of the track.
	 *
	 * @param i Index of the track in its parent sequence.
	 *
	 * @param k Cut-off index for track shortening.
	 *
	 * @param tracks Sequence containing the track to be shortened.
	 */
	void shorten(int i, int k, const Track::S &tracks);

	/**
	 * @brief Update the posterior in response to a diffusion operation performed on the given track.
	 *
	 * @param i Index of the track in its parent sequence.
	 *
	 * @param j Index of the updated pose in the track.
	 *
	 * @param tracks Sequence containing the updated track.
	 *
	 * @param readings Sequence of sensor readings over the time window.
	 */
	void diffuse(int i, int j, const Track::S &tracks);

	/**
	 * @brief Update the posterior in response to a swap operations performed on the given track pair.
	 *
	 * @param i Index to first swapped track.
	 *
	 * @param j Index to second swapped track.
	 *
	 * @param tracks Sequence containing the swapped tracks at the given indexes.
	 */
	void swap(int i, int j, const Track::S &tracks);

	/**
	 * @brief Update static readings over the time window.
	 */
	void update(const Readings &readings);
};

} // namespace virtual_scan

#endif

#ifndef UDATMO_OBSERVATION_H
#define UDATMO_OBSERVATION_H

#include "primitives.h"

#include <carmen/carmen.h>

#include <vector>

namespace udatmo
{

/**
 * @brief A single observation of a mobile obstacle.
 */
struct Observation
{
	/** @brief RDDF index of the obstacle. */
	int index;

	/** @brief Index of the lane where the observation was detected. */
	int lane;

	/** @brief Pose where the obstacle was observed. */
	carmen_position_t position;

	/** @brief Time when the obstacle was observed. */
	double timestamp;

	/**
	 * @brief Default constructor.
	 */
	Observation()
	{
		index = -1;
		lane = -1;
		position.x = 0.0;
		position.y = 0.0;
		timestamp = -1;
	}

	/**
	 * @brief Create a new mobile obstacle observation.
	 */
	Observation(int index, int lane, carmen_position_t position, double timestamp)
	{
		this->index = index;
		this->lane = lane;
		this->position = position;
		this->timestamp = timestamp;
	}
};

inline double angle(const Observation &a, const Observation &b)
{
	return angle(a.position, b.position);
}

inline double distance(const Observation &a, const Observation &b)
{
	return distance(a.position, b.position);
}

/** @brief Sequence of observations. */
typedef std::vector<Observation> Observations;

} // namespace udatmo

#endif

#ifndef UDATMO_OBSTACLE_H
#define UDATMO_OBSTACLE_H

#include "observation.h"

#include <deque>
#include <vector>

namespace udatmo
{

/**
 * @brief A moving obstacle.
 */
class Obstacle
{
	/** @brief Sequence of observations relating to this moving obstacle. */
	std::deque<Observation> track;

	/**
	 * @brief Update estimates of direction and speed of movement.
	 */
	void updateMovement();

public:
	/** @brief RDDF index of the obstacle. */
	int index;

	/** @brief Instantaneous pose of the moving obstacle. */
	carmen_ackerman_traj_point_t pose;

	/**
	 * @brief Default constructor.
	 */
	Obstacle();

	/**
	 * @brief Create a new moving obstacle from the given observation.
	 */
	Obstacle(const Observation &observation);

	/**
	 * @brief Add an observation to this moving obstacle.
	 */
	void update(const Observation &observation);

	/**
	 * @brief Return the current lane index for this obstacle.
	 */
	int lane() const;

	/**
	 * @brief Rteurn the timestamp associated with this moving obstacle.
	 */
	double timestamp() const;
};

/** @brief Sequence of moving obstacles. */
typedef std::vector<Obstacle> Obstacles;

} // namespace udatmo

#endif

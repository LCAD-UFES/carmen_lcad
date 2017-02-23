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
	std::deque<Observation> observations;

	/** @brief Number of missed observations of this moving obstacle. */
	int misses;

	/**
	 * @brief Add an observation to this moving obstacle.
	 */
	void update(const Observation &observation);

	/**
	 * @brief Update estimates of direction and speed in the direction of the robot.
	 */
	void update(const carmen_ackerman_traj_point_t &robot_pose);

public:
	/** @brief RDDF index of the obstacle. */
	int index;

	/** @brief Instantaneous pose of the moving obstacle. */
	carmen_ackerman_traj_point_t pose;

	/**
	 * @brief Default constructor.
	 */
	Obstacle();

	Obstacle(const carmen_ackerman_traj_point_t &robot_pose, Observation &observation);

	/**
	 * @brief Update this moving obstacle according to the given observation sequence.
	 */
	void update(const carmen_ackerman_traj_point_t &robot_pose, Observations &observations);

	double timestamp() const;

	bool valid() const;
};

/** @brief Sequence of moving obstacles. */
typedef std::vector<Obstacle> Obstacles;

} // namespace udatmo

#endif

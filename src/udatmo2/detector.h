#ifndef UDATMO_DETECTOR_H
#define UDATMO_DETECTOR_H

#include "obstacles.h"
#include "observation.h"
#include "udatmo_messages.h"

#include <carmen/carmen.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/rddf_messages.h>

#include <deque>
#include <vector>

#define GOAL_LIST_SIZE 1000

#define MOVING_OBJECT_HISTORY_SIZE 40

namespace udatmo
{

class Detector
{
	/** @brief System configuration settings. */
	carmen_robot_ackerman_config_t robot_config;

	/** @brief Current robot pose, speed and phi as estimated by the localization module. */
	carmen_ackerman_traj_point_t robot_pose;

	/** @brief Current map of distances between detected obstacles and plane coordinates. */
	carmen_obstacle_distance_mapper_message *current_map;

	/** @brief Latest RDDF information. */
	carmen_rddf_road_profile_message rddf;

	/** @brief Minimum number of hypothetical poses to consider ahead of the current one. */
	int min_poses_ahead;

	/** @brief Maximum number of hypothetical poses to consider ahead of the current one. */
	int max_poses_ahead;

	/** @brief History of the front moving obstacle. */
	std::deque<Observation> front_obstacle;

	int compute_num_poses_ahead();

	double speed_front();

	/**
	 * @brief Update obstacle speed estimates across its history.
	 */
	void update_moving_object_velocity();

public:
	/**
	 * @brief Default constructor.
	 */
	Detector();

	/**
	 * @brief Perform moving obstacle detection in the front of the car.
	 */
	Obstacles detect();

	/**
	 * @brief Setup detector parameters through the CARMEN parameter server.
	 */
	void setup(int argc, char *argv[]);

	/**
	 * @brief Update the current global position.
	 */
	void update(carmen_localize_ackerman_globalpos_message *msg);

	/**
	 * @brief Update the current distance map.
	 */
	void update(carmen_obstacle_distance_mapper_message *map);

	/**
	 * @brief Update the state of the RDDF encapsulated in this object.
	 */
	void update(carmen_rddf_road_profile_message *rddf);
};

/**
 * @brief Return a reference to a singleton detector instance.
 */
Detector &getDetector();

} // namespace udatmo

#endif

#ifndef UDATMO_DETECTOR_H
#define UDATMO_DETECTOR_H

#include "obstacle.h"
#include "sample_filter.h"

#include <carmen/carmen.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/rddf_messages.h>

#include <deque>

#define GOAL_LIST_SIZE 1000

#define MOVING_OBJECT_HISTORY_SIZE 40

namespace udatmo
{

class Detector
{
	/** @brief System configuration settings. */
	carmen_robot_ackerman_config_t robot_config;

	/** @brief Current map of distances between detected obstacles and plane coordinates. */
	carmen_obstacle_distance_mapper_message *current_map;

	/** @brief Current vehicle pose, speed and phi as estimated by the localization module. */
	carmen_ackerman_traj_point_t current_pose;

	// TODO: Confirm this attribute can be fused with current_pose
	carmen_ackerman_traj_point_t robot_pose;


	/** @brief Latest RDDF information. */
	carmen_rddf_road_profile_message rddf;

	/** @brief Minimum number of hypothetical poses to consider ahead of the current one. */
	int min_poses_ahead;

	/** @brief Maximum number of hypothetical poses to consider ahead of the current one. */
	int max_poses_ahead;

	/** @brief History of the front moving obstacle. */
	std::deque<Obstacle> front_obstacle;

	int compute_num_poses_ahead();

	double speed_front();

	/**
	 * @brief Update obstacle speed estimates across its history.
	 */
	void update_moving_object_velocity();

	/**
	 * @brief Publish a detection message using the current detection data.
	 */
	void publish(int rddf_index);

public:

	SampleFilter speed;

	/**
	 * @brief Default constructor.
	 */
	Detector();

	/**
	 * @brief Perform moving obstacle detection in the front of the car.
	 */
	void detect();

	/**
	 * @brief Setup detector parameters through the CARMEN parameter server.
	 */
	void setup(int argc, char *argv[]);

	/**
	 * @brief Update the current distance map.
	 */
	void update(carmen_obstacle_distance_mapper_message *map);

	/**
	 * @brief Update the current global position.
	 */
	void update(carmen_localize_ackerman_globalpos_message *msg);

	/**
	 * @brief Update the state of the RDDF encapsulated in this object.
	 */
	void update(carmen_rddf_road_profile_message *rddf);
};

} // namespace udatmo

#endif

#ifndef DATOMIC_DETECTOR_H
#define DATOMIC_DETECTOR_H

#include "obstacle.h"
#include "sample_filter.h"

#include <carmen/carmen.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/rddf_messages.h>

namespace udatmo
{

#define MOVING_OBJECT_HISTORY_SIZE 40

class Detector
{
	/**
	 * @brief Update obstacle speed estimates across its history.
	 */
	void update_moving_object_velocity(carmen_ackerman_traj_point_t &robot_pose);

	inline bool set_detected(bool value)
	{
		detected = value;
		return detected;
	}

public:
	/** @brief System configuration settings. */
	carmen_robot_ackerman_config_t robot_config;

	/** @brief Result of last detection operation. */
	bool detected;

	/** @brief History of the front moving obstacle. */
	Obstacle moving_object[MOVING_OBJECT_HISTORY_SIZE];

	bool obstacle_already_detected;

	SampleFilter speed;

	/**
	 * @brief Create a new moving obstacle detector.
	 */
	Detector(const carmen_robot_ackerman_config_t &robot_config);

	/**
	 * @brief Perform moving obstacle detection in the front of the car.
	 */
	int detect(carmen_obstacle_distance_mapper_map_message *current_map,
				carmen_rddf_road_profile_message *rddf,
				int goal_index,
				int rddf_pose_index,
				carmen_robot_and_trailer_traj_point_t robot_pose,
				double circle_radius,
				double timestamp);

	int detect(carmen_obstacle_distance_mapper_map_message *current_map,
				carmen_rddf_road_profile_message *rddf,
				int goal_index,
				int rddf_pose_index,
				carmen_robot_and_trailer_traj_point_t robot_pose,
				double circle_radius,
				double displacement,
				double timestamp);

	void copy_state(Detector *detector);

	/**
	 * @brief Shift the obstacle history one position towards the back.
	 */
	void shift();

	double speed_front();

	carmen_ackerman_traj_point_t get_moving_obstacle_position(void);
	double get_moving_obstacle_distance(carmen_robot_and_trailer_traj_point_t robot_pose);
};

} // namespace udatmo

#endif

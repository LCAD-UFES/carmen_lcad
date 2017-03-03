#ifndef UDATMO_DATMO_H
#define UDATMO_DATMO_H

#include "obstacle.h"

#include <carmen/carmen.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/rddf_messages.h>

#include <opencv2/opencv.hpp>


namespace udatmo
{


class DATMO
{
	/** @brief System configuration settings. */
	carmen_robot_ackerman_config_t robot_config;

	/** @brief Minimum number of hypothetical poses to consider ahead of the current one. */
	int min_poses_ahead;

	/** @brief Maximum number of hypothetical poses to consider ahead of the current one. */
	int max_poses_ahead;

	/** @brief Current robot pose, speed and phi as estimated by the localization module. */
	carmen_ackerman_traj_point_t robot_pose;

	/** @brief Current map of distances between detected obstacles and plane coordinates. */
	carmen_obstacle_distance_mapper_message *current_map;

	/** @brief Latest RDDF information. */
	carmen_rddf_road_profile_message rddf;

	/** @brief Sequence of observed obstacles. */
	Observations observations;

	/** @brief Sequence of detected obstacles. */
	Obstacles obstacles;

	/**
	 * @brief Assign observation `j` either to an existing or new moving obstacle.
	 */
	void assign(int j, const cv::Mat assignments, Obstacles &recognized);

	/**
	 * @brief Scan the given pose array for moving obstacle observations.
	 */
	void detect(carmen_ackerman_traj_point_t *poses, int n);

	/**
	 * @brief Scan the current input for moving obstacle observations.
	 */
	void detect();

	/**
	 * @brief Compute the distances between known obstacles and latest observations.
	 */
	cv::Mat distances() const;

	/**
	 * @brief Compute the number of RDDF poses ahead of the robot that must be considered.
	 */
	int posesAhead() const;

public:
	/**
	 * @brief Default constructor.
	 */
	DATMO();

	/**
	 * @brief Perform moving obstacle detection.
	 */
	const Obstacles &track();

	/**
	 * @brief Setup DATMO parameters through the CARMEN parameter server.
	 */
	void setup(int argc, char *argv[]);

	/**
	 * @brief Setup DATMO parameters.
	 */
	void setup(const carmen_robot_ackerman_config_t &robot_config, int min_poses_ahead, int max_poses_ahead);

	/**
	 * @brief Update the current global position.
	 */
	void update(const carmen_ackerman_traj_point_t &robot_pose);

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
 * @brief Return a reference to a singleton DATMO instance.
 */
DATMO &getDATMO();


} // namespace udatmo


#endif

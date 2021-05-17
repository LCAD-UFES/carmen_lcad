/*
 * util.h
 *
 *  Created on: 01/02/2012
 *      Author: rradaelli
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <carmen/carmen.h>
#include "../model/rrt_node.h"
#include "../model/robot_config.h"
#include "../model/command.h"
#include "../model/pose.h"
#include "../model/cost_map.h"

class Util
{
public:
	/**
	 * Return true if the x, y is valid in the map
	 * otherwise return false
	 */
	static bool is_valid_position(const int &x, const int &y, const carmen_map_config_t &map_config);
	static bool is_valid_position(const int &x, const int &y);
	static bool is_valid_position(const Pose &pose, const carmen_map_config_t &map_config);
	static bool is_valid_position(const Pose &pose);

	/*
	 * Measure the alignment of the pose in relation to target_pose
	 * The return is between 0 and 1, if the pose is perfectly aligned the return is 0
	 */
	static double heading(const Pose &pose, const Pose &target_pose);

	/**
	 * Return true if p2 is behind p1
	 */
	static bool is_behind(const Pose &p1, const Pose &p2);

	static double heading2(const Pose &pose, const Pose &target_pose);

	/*
	 * Return a random number between 0 and 1
	 */
	static double normalized_random();

	/**
	 * Return a random pose in the map
	 */
	static Pose random_pose();

	/**
	 * Return a valid random command
	 */
	static Command random_command();

	/**
	 * Convert an world pose to a pose in the resolution of the map
	 */
	static Pose to_map_pose(carmen_point_t world_pose);
	static Pose to_map_pose(const Pose &world_pose, const carmen_map_config_t &map_config);
	static Pose to_map_pose(const Pose &world_pose);


	/**
	 * Convert the coordinates of map pose to meters
	 */
	static carmen_point_t to_world_pose(Pose map_pose);
	static Pose to_global_pose(const Pose &map_pose);
	static Pose to_global_pose(const Pose &map_pose, const carmen_map_config_t &map_config);

	/**
	 * Convert the map unit length to meter
	 */
	static double to_meter(const double &map_unit);

	/**
	 * Convert meter to the map unit length
	 */
	static double to_map_unit(const double &meter);

	static double to_map_unit(const double &meter, const carmen_map_config_t &map_config);

	/**
	 * Return the current time in seconds
	 */
	static double get_time();

	/**
	 * Return 1 if the number is bigger than zero, -1 if is lesser the zero, otherwise return zero
	 */
	static int signalz(double num);

	/**
	 * Return -1 if num is lesser then zero otherwise return 1
	 */
	static int signal(double num);

	static bool can_reach(Pose robot, Pose goal);

	static Pose convert_to_pose(const carmen_point_t carmen_pose, double beta);

	static carmen_point_t convert_to_carmen_point_t(const Pose pose);
};

#endif /* UTIL_H_ */

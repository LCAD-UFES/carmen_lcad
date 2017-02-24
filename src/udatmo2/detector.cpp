#include "detector.h"

#include "logging.h"
#include "primitives.h"
#include "udatmo_interface.h"

#include <carmen/collision_detection.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

namespace udatmo
{

Detector &getDetector() {
	static Detector *detector = NULL;
	if (detector == NULL)
	{
		CARMEN_LOG_TO_FILE("udatmo.log");
		detector = new Detector();
	}

	return *detector;
}


Detector::Detector():
	min_poses_ahead(0),
	max_poses_ahead(0),
	current_map(NULL)
{
	memset(&robot_config, 0, sizeof(carmen_robot_ackerman_config_t));
	memset(&robot_pose, 0, sizeof(carmen_ackerman_traj_point_t));
	memset(&rddf, 0, sizeof(carmen_rddf_road_profile_message));

	robot_pose.v = nan("");
}


void Detector::observate()
{
	double near = robot_config.distance_between_front_and_rear_axles + 1.5;
	double front = robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels;
	double within = robot_config.width / 2.0;

	CARMEN_LOG(trace, "Observation start");

	observations.clear();
	for (int i = 0, n = rddf.number_of_poses; i < n; i++)
	{
		if (distance(rddf.poses[i], robot_pose) < near)
			continue;

		carmen_point_t front_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&rddf.poses[i], front);
		carmen_position_t position = carmen_obstacle_avoider_get_nearest_obstacle_cell_from_global_point(&front_car_pose, current_map);
		if (distance(position, front_car_pose) < within)
		{
			observations.push_back(Observation(i, position, rddf.timestamp));
			CARMEN_LOG(trace,
				"Observation"
				<< ": t = " << rddf.timestamp
				<< ", position = (" << position.x << ", " << position.y << ")"
				<< ", rddf = " << i
			);

			break;
		}
	}

	CARMEN_LOG(trace, "Observations total: " << observations.size());
}


struct assign
{
	const carmen_ackerman_traj_point_t &robot_pose_;

	Observations &observations_;

	assign(const carmen_ackerman_traj_point_t &robot_pose, Observations &observations):
		robot_pose_(robot_pose),
		observations_(observations)
	{
		// Nothing to do.
	}

	bool operator () (Obstacle &obstacle)
	{
		obstacle.update(robot_pose_, observations_);
		return !obstacle.valid();
	}
};

const Obstacles &Detector::detect()
{
	CARMEN_LOG(trace, "Obstacle update start");

	if (isnan(robot_pose.v) || current_map == NULL || rddf.number_of_poses == 0)
	{
		CARMEN_LOG(trace, "Obstacle update aborted (preconditions not met)");
		return obstacles;
	}

	// Check the map for obstacle points in the focus regions.
	observate();

	// Update current moving obstacles with the observations, removing stale cases.
	Obstacles::iterator n = obstacles.end();
	Obstacles::iterator i = std::remove_if(obstacles.begin(), n, assign(robot_pose, observations));
	obstacles.erase(i, n);

	CARMEN_LOG(trace, "Observations not assigned: " << observations.size());

	// Instantiate new moving obstacles from observations not associated to current obstacles.
	for (Observations::iterator i = observations.begin(), n = observations.end(); i != n; ++i)
		obstacles.push_back(Obstacle(robot_pose, *i));

	CARMEN_LOG(trace, "Obstacles total: " << obstacles.size());

	return obstacles;
}


void Detector::setup(int argc, char *argv[])
{
	carmen_param_t param_list[] =
	{
		{(char*) "robot", (char*) "max_v", CARMEN_PARAM_DOUBLE, &robot_config.max_v, 1, NULL},
		{(char*) "robot", (char*) "max_steering_angle", CARMEN_PARAM_DOUBLE, &robot_config.max_phi, 1, NULL},
		{(char*) "robot", (char*) "length", CARMEN_PARAM_DOUBLE, &robot_config.length, 0, NULL},
		{(char*) "robot", (char*) "width", CARMEN_PARAM_DOUBLE, &robot_config.width, 0, NULL},
		{(char*) "robot", (char*) "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward, 1, NULL},
		{(char*) "robot", (char*) "maximum_deceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_forward, 1, NULL},
		{(char*) "robot", (char*) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 1, NULL},
		{(char*) "robot", (char*) "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
		{(char*) "robot", (char*) "distance_between_front_car_and_front_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_car_and_front_wheels, 1, NULL},
		{(char*) "behavior_selector", (char*) "rddf_num_poses_ahead_min", CARMEN_PARAM_INT, &min_poses_ahead, 0, NULL},
		{(char*) "behavior_selector", (char*) "rddf_num_poses_ahead_limit", CARMEN_PARAM_INT, &max_poses_ahead, 0, NULL}
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}


void Detector::setup(const carmen_robot_ackerman_config_t &robot_config, int min_poses_ahead, int max_poses_ahead)
{
	this->robot_config = robot_config;
	this->min_poses_ahead = min_poses_ahead;
	this->max_poses_ahead = max_poses_ahead;
}


void Detector::update(const carmen_ackerman_traj_point_t &robot_pose)
{
	this->robot_pose = robot_pose;
}


void Detector::update(carmen_obstacle_distance_mapper_message *map)
{
	current_map = map;
}


int Detector::posesAhead() const
{
	int num_poses_ahead = min_poses_ahead;
	double common_goal_v = 3.0;

	if (common_goal_v < robot_pose.v)
	{
		double distance = robot_pose.v * 6.5;
		if (distance > 0)
			num_poses_ahead = (distance / 0.5) + 1;
	}

	if (num_poses_ahead < min_poses_ahead)
		return min_poses_ahead;

	if (num_poses_ahead > max_poses_ahead)
		return max_poses_ahead;

	return num_poses_ahead;
}


void Detector::update(carmen_rddf_road_profile_message *rddf_msg)
{
	int num_poses_ahead = rddf_msg->number_of_poses;
	if (!(0 < num_poses_ahead && num_poses_ahead < min_poses_ahead))
		num_poses_ahead = posesAhead();

	if (rddf.number_of_poses != num_poses_ahead)
	{
		rddf.number_of_poses = num_poses_ahead;
		resize(rddf.poses, rddf.number_of_poses);
		resize(rddf.annotations, rddf.number_of_poses);
	}

	if (rddf_msg->number_of_poses_back > 0)
	{
		rddf.number_of_poses_back = num_poses_ahead;
		resize(rddf.poses_back, rddf.number_of_poses_back);
	}

	rddf.timestamp = rddf_msg->timestamp;
	memcpy(rddf.poses, rddf_msg->poses, sizeof(carmen_ackerman_traj_point_t) * rddf.number_of_poses);
	memcpy(rddf.poses_back, rddf_msg->poses_back, sizeof(carmen_ackerman_traj_point_t) * rddf.number_of_poses_back);
	memcpy(rddf.annotations, rddf_msg->annotations, sizeof(int) * rddf.number_of_poses);
}

} // namespace udatmo

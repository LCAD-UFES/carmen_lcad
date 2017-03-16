#include "datmo.h"

#include "logging.h"
#include "munkres.h"
#include "primitives.h"

#include <carmen/collision_detection.h>

#include <boost/bind.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>


namespace udatmo
{


DATMO &getDATMO() {
	static DATMO *datmo = NULL;
	if (datmo == NULL)
	{
		CARMEN_LOG_TO_FILE("udatmo.log");
		datmo = new DATMO();
	}

	return *datmo;
}


DATMO::DATMO():
	min_poses_ahead(0),
	max_poses_ahead(0),
	current_map(NULL)
{
	clear(robot_config);
	clear(origin);
	clear(robot_pose);
	clear(rddf);

	origin.x = nan("");
	robot_pose.v = nan("");
}


void DATMO::detect(carmen_ackerman_traj_point_t *poses, int n)
{
	double width = robot_config.width;
	double lanes[] = {0, 1.5 * width, -1.5 * width};
	double near = robot_config.distance_between_front_and_rear_axles + 1.5;
	double front = robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels;
	double within = width / 2.0;

// 	for (int l = 0; l < 3; l++)
	int l = 0;
	{
		double lane = lanes[l];
		for (int i = 0; i < n; i++)
		{
			carmen_ackerman_traj_point_t pose = poses[i];

			// Displaces the pose sideways according to the lane currently being scanned.
			double theta = pose.theta;
			pose.x += lane * sin(theta); // Trigonometric functions are inverted on purpose, to
			pose.y += lane * cos(theta); // compute a displacement sideways to the pose direction.

			// Avoid using poses that overlap with the robot's current pose.
			if (lane == 0 && distance(pose, robot_pose) < near)
				continue;

			carmen_point_t front_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&pose, front);
			carmen_position_t position = carmen_obstacle_avoider_get_nearest_obstacle_cell_from_global_point(&front_car_pose, current_map);
			if (distance(position, front_car_pose) < within)
			{
				observations.push_back(Observation(i, position, rddf.timestamp));
				CARMEN_LOG(trace,
					"Observation l = " << l
					<< ": t = " << rddf.timestamp - carmen_ipc_initialize_time()
					<< ", position = " << relative_xy(position, origin)
					<< ", distance = " << distance(position, robot_pose)
					<< ", rddf = " << i
				);

				break;
			}
		}
	}
}


void DATMO::detect()
{
	observations.clear();

	CARMEN_LOG(trace, "Observations front");

	detect(rddf.poses, rddf.number_of_poses);

// 	CARMEN_LOG(trace, "Observations back");
//
// 	detect(rddf.poses_back, rddf.number_of_poses_back);

	CARMEN_LOG(trace, "Observations total: " << observations.size());
}


cv::Mat DATMO::distances() const
{
	int rows = obstacles.size();
	int cols = observations.size();
	cv::Mat distances(rows, cols, CV_64F);
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
			distances.at<double>(i, j) = distance(obstacles[i].pose, observations[j].position);

	CARMEN_LOG(trace, "Distances: " << distances);

	return distances;
}


const Obstacles &DATMO::track()
{
	static const double MAX_DISTANCE = 20.0;

	CARMEN_LOG(trace, "Obstacle update start");

	tracking.clear(); // Clear the list of tracking obstacles ahead of new detection cycle.
	if (isnan(robot_pose.v) || current_map == NULL || rddf.number_of_poses == 0)
	{
		CARMEN_LOG(trace, "Obstacle update aborted (preconditions not met)");
		return tracking;
	}

	detect(); // Check the map for obstacle points in the focus regions.
	if (obstacles.size() == 0)
	{
		CARMEN_LOG(trace, "No known obstacles, convert " << observations.size() << " observations to obstacles");
		for (Observations::const_iterator i = observations.begin(), n = observations.end(); i != n; ++i)
			obstacles.push_back(Obstacle(*i));

		return tracking;
	}
	else if (observations.size() == 0)
	{
		CARMEN_LOG(trace, "No observations found");
		obstacles.clear();
		return tracking;
	}

	cv::Mat assignments = munkres(distances(), MAX_DISTANCE);
	CARMEN_LOG(trace, "Assignments: " << assignments);

	Obstacles assigned;
	assigned.reserve(obstacles.size());
	for (int j = 0, n = observations.size(); j < n; j++)
		assign(j, assignments, assigned);

	obstacles = assigned;

	CARMEN_LOG(trace, "Obstacles found: " << obstacles.size());
	CARMEN_LOG(trace, "Obstacles tracked: " << tracking.size());

	return tracking;
}


void DATMO::assign(int j, const cv::Mat assignments, Obstacles &assigned)
{
	int rows = obstacles.size();
	for (int i = 0; i < rows; i++)
	{
		if (assignments.at<int>(i, j) == STAR)
		{
			Obstacle &obstacle = obstacles[i];
			obstacle.update(observations[j]);
			if (obstacle.pose.v > 0.01)
				tracking.push_back(obstacle);

			assigned.push_back(obstacle);
			CARMEN_LOG(trace, "Obstacle #" << i << ", observation #" << j << ", v=" << obstacle.pose.v << ", theta=" << obstacle.pose.theta);
			return;
		}
	}

	// Add new moving obstacles.
	int side = assignments.rows;
	for (int i = rows; i < side; i++)
	{
		if (assignments.at<int>(i, j) == STAR)
		{
			assigned.push_back(Obstacle(observations[j]));
			return;
		}
	}
}


void DATMO::setup(int argc, char *argv[])
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


void DATMO::setup(const carmen_robot_ackerman_config_t &robot_config, int min_poses_ahead, int max_poses_ahead)
{
	this->robot_config = robot_config;
	this->min_poses_ahead = min_poses_ahead;
	this->max_poses_ahead = max_poses_ahead;
}


void DATMO::update(const carmen_ackerman_traj_point_t &robot_pose)
{
	this->robot_pose = robot_pose;
	if (isnan(origin.x))
		origin = robot_pose;

	CARMEN_LOG(trace, "Robot pose = " << relative_xyt(robot_pose, origin));
}


void DATMO::update(carmen_obstacle_distance_mapper_message *map)
{
	current_map = map;
}


int DATMO::posesAhead() const
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


void DATMO::update(carmen_rddf_road_profile_message *rddf_msg)
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

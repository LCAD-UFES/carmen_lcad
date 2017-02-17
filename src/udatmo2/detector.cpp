#include "detector.h"

#include "udatmo_interface.h"
#include "udatmo_memory.h"
#include <carmen/collision_detection.h>
#include <cmath>

namespace udatmo
{

Detector::Detector()
{
	memset(&robot_config, 0, sizeof(carmen_robot_ackerman_config_t));
	memset(&rddf, 0, sizeof(carmen_rddf_road_profile_message));
	current_map = NULL;
}


double
Detector::speed_front()
{
	double average_v = 0.0;
	double count = 0.0;
	for (int i = front_obstacle.size() - 2; i >= 0 ; i--)
	{
		if (front_obstacle[i].pose.v > -0.0001)
		{
			average_v += front_obstacle[i].pose.v;
			count += 1.0;
		}
	}

	if (count > 0.0)
		average_v /= count;

//	return speed();
	return (average_v);
}


void
Detector::update_moving_object_velocity()
{
	// TODO: determine if the speed really has to be recalculated for the entire
	// history every time a new observation is added.

	double average_v = 0.0;
	double count = 0.0;
	for (int i = front_obstacle.size() - 2; i >= 0 ; i--)
	{
		double dist = carmen_distance_ackerman_traj(&front_obstacle[i].pose, &front_obstacle[i + 1].pose);
		// distance in the direction of the robot: https://en.wikipedia.org/wiki/Vector_projection
		double angle = atan2(front_obstacle[i].pose.y - front_obstacle[i + 1].pose.y, front_obstacle[i].pose.x - front_obstacle[i + 1].pose.x);
		double distance = dist * cos(angle - robot_pose.theta);
		double delta_t = front_obstacle[i].timestamp - front_obstacle[i + 1].timestamp;

		double v = -1.0; // invalid v
		if (delta_t > 0.01 && delta_t < 0.2)
			v = distance / delta_t;
		if (v > 60.0)
			v = -1.0;
		if (v > -0.00001)
		{
			average_v += v;
			count += 1.0;
		}

		front_obstacle[i].pose.v = v;
	}

	if (count > 0.0)
		average_v /= count;
}


void
Detector::detect()
{
	int goal_index = 0; // TODO: verify whether / how to increment goal_index, and if negative, remove it.
	double circle_radius = (robot_config.width + 0.0) / 2.0; // metade da largura do carro + um espacco de guarda
	for (int rddf_pose_index = 0; rddf_pose_index < rddf.number_of_poses && goal_index < GOAL_LIST_SIZE; rddf_pose_index++)
	{
		if (carmen_distance_ackerman_traj(&rddf.poses[rddf_pose_index], &robot_pose) < robot_config.distance_between_front_and_rear_axles + 1.5)
			continue;

		double disp = robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels;
		carmen_point_t front_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&rddf.poses[rddf_pose_index], disp);
		carmen_position_t obstacle = carmen_obstacle_avoider_get_nearest_obstacle_cell_from_global_point(&front_car_pose, current_map);
		double distance = sqrt(
			(front_car_pose.x - obstacle.x) * (front_car_pose.x - obstacle.x) +
			(front_car_pose.y - obstacle.y) * (front_car_pose.y - obstacle.y)
		);

		if (distance < circle_radius)
		{
			Obstacle observed(goal_index, obstacle, robot_pose, rddf.timestamp);
			front_obstacle.push_front(observed);
			while (front_obstacle.size() > MOVING_OBJECT_HISTORY_SIZE)
				front_obstacle.pop_back();

			update_moving_object_velocity();
			speed += observed.pose.v;
			publish(rddf_pose_index);
			return;
		}
	}

	publish(-1);
}


void
Detector::publish(int rddf_index)
{
	static carmen_udatmo_moving_obstacles_message *message = NULL;
	if (message == NULL)
		message = carmen_udatmo_new_moving_obstacles_message(1);

	message->timestamp = rddf.timestamp;

	Obstacle &front_observed = front_obstacle[0];
	carmen_datmo_moving_obstacle &front_obstacle = message->obstacles[0];
	front_obstacle.rddf_index = rddf_index;
	front_obstacle.x = front_observed.pose.x;
	front_obstacle.y = front_observed.pose.y;
	front_obstacle.theta = front_observed.pose.theta;
	front_obstacle.v = speed_front();

	carmen_udatmo_publish_moving_obstacles_message(message);
}


void
Detector::setup(int argc, char *argv[])
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


int
Detector::compute_num_poses_ahead()
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


void
Detector::update(carmen_obstacle_distance_mapper_message *map)
{
	current_map = map;
}


void
Detector::update(carmen_localize_ackerman_globalpos_message *msg)
{
	robot_pose.x = msg->globalpos.x;
	robot_pose.y = msg->globalpos.y;
	robot_pose.theta = msg->globalpos.theta;
	robot_pose.v = msg->v;
	robot_pose.phi = msg->phi;

	// TODO: check if this the best place to call this function, if all dependencies are met.
	detect();
}


void
Detector::update(carmen_rddf_road_profile_message *rddf_msg)
{
	int num_poses_ahead = rddf_msg->number_of_poses;
	if (!(0 < num_poses_ahead && num_poses_ahead < min_poses_ahead))
		num_poses_ahead = compute_num_poses_ahead();

	if (rddf.number_of_poses != num_poses_ahead)
	{
		rddf.number_of_poses = num_poses_ahead;
		RESIZE(rddf.poses, carmen_ackerman_traj_point_t, rddf.number_of_poses);
		RESIZE(rddf.annotations, int, rddf.number_of_poses);
	}

	if (rddf_msg->number_of_poses_back > 0)
	{
		rddf.number_of_poses_back = num_poses_ahead;
		RESIZE(rddf.poses_back, carmen_ackerman_traj_point_t, rddf.number_of_poses_back);
	}

	rddf.timestamp = rddf_msg->timestamp;
	memcpy(rddf.poses, rddf_msg->poses, sizeof(carmen_ackerman_traj_point_t) * rddf.number_of_poses);
	memcpy(rddf.poses_back, rddf_msg->poses_back, sizeof(carmen_ackerman_traj_point_t) * rddf.number_of_poses_back);
	memcpy(rddf.annotations, rddf_msg->annotations, sizeof(int) * rddf.number_of_poses);
}

} // namespace udatmo

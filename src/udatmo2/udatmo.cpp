#include "udatmo.h"

#include "detector.h"
#include "primitives.h"
#include "udatmo_interface.h"

using udatmo::Obstacle;
using udatmo::Obstacles;
using udatmo::resize;
using udatmo::distance;
using udatmo::getDetector;

#define NUM_OBSTACLES 1

void carmen_udatmo_init(carmen_robot_ackerman_config_t *robot_config, int min_poses_ahead, int max_poses_ahead)
{
	getDetector().setup(*robot_config, min_poses_ahead, max_poses_ahead);
}

void carmen_udatmo_setup(int argc, char *argv[])
{
	getDetector().setup(argc, argv);
}

static carmen_udatmo_moving_obstacles_message *carmen_udatmo_detector_message(void)
{
	static carmen_udatmo_moving_obstacles_message *message = NULL;
	if (message == NULL)
		message = carmen_udatmo_new_moving_obstacles_message(NUM_OBSTACLES);

	return message;
}

static void carmen_udatmo_resize_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message, int size)
{
	if (message->num_obstacles == size)
		return;

	message->num_obstacles = size;
	resize(message->obstacles, size);
}

static void carmen_udatmo_set_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message, const Obstacles &obstacles)
{
	int n = obstacles.size();
	carmen_udatmo_resize_moving_obstacles_message(message, n);
	for (int i = 0; i < n; i++)
	{
		carmen_udatmo_moving_obstacle &entry = message->obstacles[i];
		const Obstacle &obstacle = obstacles[i];
		entry.rddf_index = obstacle.index;
		entry.x = obstacle.pose.x;
		entry.y = obstacle.pose.y;
		entry.theta = obstacle.pose.theta;
		entry.v = obstacle.pose.v;
	}
}

static void carmen_udatmo_reset_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message)
{
	carmen_udatmo_resize_moving_obstacles_message(message, NUM_OBSTACLES);
	for (int i = 0; i < NUM_OBSTACLES; i++)
	{
		memset(message->obstacles + i, 0, sizeof(carmen_udatmo_moving_obstacle));
		message->obstacles[0].rddf_index = -1;
	}
}

carmen_udatmo_moving_obstacles_message *carmen_udatmo_detect_moving_obstacles(void)
{
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_detector_message();

	const Obstacles &obstacles = getDetector().detect();
	if (obstacles.size() > 0)
		carmen_udatmo_set_moving_obstacles_message(message, obstacles);
	else
		carmen_udatmo_reset_moving_obstacles_message(message);

	return message;
}

int carmen_udatmo_front_obstacle_detected(void)
{
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_detector_message();
	return (message->obstacles[0].rddf_index != -1);
}

double carmen_udatmo_front_obstacle_speed(void)
{
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_detector_message();
	return message->obstacles[0].v;
}

double carmen_udatmo_front_obstacle_distance(carmen_ackerman_traj_point_t *robot_pose)
{
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_detector_message();
	const carmen_udatmo_moving_obstacle &obstacle = message->obstacles[0];

	return distance(*robot_pose, obstacle);
}

void carmen_udatmo_update_distance_map(carmen_obstacle_distance_mapper_message *message)
{
	getDetector().update(message);
}

void carmen_udatmo_update_robot_pose_with_globalpos(carmen_localize_ackerman_globalpos_message *message)
{
	carmen_ackerman_traj_point_t robot_pose;
	robot_pose.x = message->globalpos.x;
	robot_pose.y = message->globalpos.y;
	robot_pose.theta = message->globalpos.theta;
	robot_pose.v = message->v;
	robot_pose.phi = message->phi;

	getDetector().update(robot_pose);
}

void carmen_udatmo_update_robot_pose_with_truepos(carmen_simulator_ackerman_truepos_message *message)
{
	carmen_ackerman_traj_point_t robot_pose;
	robot_pose.x = message->truepose.x;
	robot_pose.y = message->truepose.y;
	robot_pose.theta = message->truepose.theta;
	robot_pose.v = message->v;
	robot_pose.phi = message->phi;

	getDetector().update(robot_pose);
}

void carmen_udatmo_update_rddf(carmen_rddf_road_profile_message *message)
{
	getDetector().update(message);
}

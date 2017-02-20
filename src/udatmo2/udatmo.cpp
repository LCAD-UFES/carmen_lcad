#include "udatmo.h"

#include "detector.h"
#include "udatmo_interface.h"

using udatmo::getDetector;
using udatmo::Obstacles;

static carmen_udatmo_moving_obstacles_message *carmen_udatmo_detector_message(void)
{
	static carmen_udatmo_moving_obstacles_message *message = NULL;
	if (message == NULL)
		message = carmen_udatmo_new_moving_obstacles_message(NUM_OBSTACLES);

	return message;
}

carmen_udatmo_moving_obstacles_message *carmen_udatmo_detect_moving_obstacles(void)
{
	Obstacles obstacles = getDetector().detect();
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_detector_message();
	memcpy(message->obstacles, &(obstacles[0]), NUM_OBSTACLES * sizeof(carmen_udatmo_moving_obstacle));
	message->timestamp = obstacles.timestamp;

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
	const carmen_udatmo_moving_obstacle obstacle = message->obstacles[0];

	double dx = robot_pose->x - obstacle.x;
	double dy = robot_pose->y - obstacle.y;
	return sqrt(dx*dx + dy*dy);
}

void carmen_udatmo_setup(int argc, char *argv[])
{
	getDetector().setup(argc, argv);
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

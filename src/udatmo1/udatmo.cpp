#include "udatmo.h"

#include "detector.h"

#include <stdexcept>

using udatmo::getDetector;

void carmen_udatmo_init(carmen_robot_ackerman_config_t *robot_config, int min_poses_ahead, int max_poses_ahead)
{
	getDetector().setup(*robot_config, min_poses_ahead, max_poses_ahead);
}

void carmen_udatmo_setup(int argc, char *argv[])
{
	getDetector().setup(argc, argv);
}

int carmen_udatmo_front_obstacle_detected(void)
{
	return (getDetector().detected);
}

carmen_udatmo_moving_obstacles_message *carmen_udatmo_detect_moving_obstacles(void)
{
	return getDetector().detect();
}

double carmen_udatmo_front_obstacle_speed(carmen_ackerman_traj_point_t * /*robot_pose*/)
{
	return getDetector().speed_front();
}

carmen_ackerman_traj_point_t udatmo_get_moving_obstacle_position(void)
{
	return getDetector().get_moving_obstacle_position();
}

double carmen_udatmo_front_obstacle_distance(carmen_ackerman_traj_point_t *robot_pose)
{
	return getDetector().get_moving_obstacle_distance(robot_pose);
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

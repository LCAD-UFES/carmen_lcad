#include "udatmo.h"

#include "detector.h"

#include <stdexcept>

using udatmo::Detector;

static Detector *detector = NULL;

void udatmo_init(const carmen_robot_ackerman_config_t robot_config)
{
	if (detector != NULL)
		throw std::runtime_error("uDATMO module already initialized");

	detector = new Detector(robot_config);
}

int udatmo_obstacle_detected(void)
{
	return (detector->detected ? 1 : -1);
}

void udatmo_clear_detected(void)
{
	detector->detected = false;
}

void udatmo_shift_history(void)
{
	detector->shift();
}

int udatmo_detect_obstacles(carmen_obstacle_distance_mapper_message *current_map,
							carmen_rddf_road_profile_message *rddf,
							int goal_index,
							int rddf_pose_index,
							carmen_ackerman_traj_point_t car_pose,
							carmen_ackerman_traj_point_t robot_pose,
							double timestamp)
{
	detector->detect(current_map, rddf, goal_index, rddf_pose_index, car_pose, robot_pose, timestamp);

	return udatmo_obstacle_detected();
}

double udatmo_speed_front(void)
{
	return detector->speed_front();
}

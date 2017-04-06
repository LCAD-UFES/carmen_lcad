#include "udatmo.h"

#include "datmo.h"
#include "udatmo_api.h"
#include "udatmo_interface.h"

#include <carmen/map_server_interface.h>

#include <stdexcept>

using udatmo::getDATMO;

void udatmo_init(const carmen_robot_ackerman_config_t robot_config)
{
	carmen_udatmo_init(&robot_config);

	// Subscribe to offline map messages.
	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) carmen_udatmo_update_offline_map, CARMEN_SUBSCRIBE_LATEST);
}

bool udatmo_obstacle_detected(double timestamp)
{
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_get_moving_obstacles();
	carmen_udatmo_moving_obstacle *front_obstacle = carmen_udatmo_find_front_moving_obstacle(message);
	return (front_obstacle->rddf_index != -1 && timestamp - message->timestamp < 10.0);
}

void udatmo_clear_detected(void)
{
	// Nothing to do.
}

void udatmo_shift_history(void)
{
	// Nothing to do.
}

int udatmo_detect_obstacle_index(carmen_obstacle_distance_mapper_message *current_map,
							carmen_rddf_road_profile_message *rddf,
							int goal_index,
							int /*rddf_pose_index*/,
							carmen_ackerman_traj_point_t robot_pose,
							double timestamp)
{
	// Update DATMO state.
	carmen_udatmo_update_robot_pose(&robot_pose);
	carmen_udatmo_update_distance_map(current_map);
	carmen_udatmo_update_rddf(rddf);

	// Check for moving obstacles only if the last detection was from a different
	// (probably earlier) timestamp.
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_get_moving_obstacles();
	if (message->timestamp != timestamp)
	{
		message = carmen_udatmo_detect_moving_obstacles();

		// TODO: check if this is really necessary, or we can trust the argument
		// timestamp is truly the "current" timestamp.
		message->timestamp = timestamp;

		carmen_udatmo_display_moving_obstacles_message(message, &(getDATMO().robot_config));
	}

	// Abort the operation only after searching for moving obstacles, so it won't
	// be too long between updates.
	if (goal_index != 0)
		return -1;

	// This will return -1 if moving obstacles were found, but none in front of the car.
	carmen_udatmo_moving_obstacle *front_obstacle = carmen_udatmo_find_front_moving_obstacle(message);
	return front_obstacle->rddf_index;
}

double udatmo_speed_front(void)
{
	return carmen_udatmo_front_obstacle_speed(&(getDATMO().robot_pose));
}

carmen_ackerman_traj_point_t udatmo_get_moving_obstacle_position(void)
{
	return carmen_udatmo_front_obstacle_position();
}

double udatmo_get_moving_obstacle_distance(carmen_ackerman_traj_point_t robot_pose, carmen_robot_ackerman_config_t *robot_config)
{
	double offset = robot_config->distance_between_front_and_rear_axles + robot_config->distance_between_front_car_and_front_wheels;
	double distance = carmen_udatmo_front_obstacle_distance(&robot_pose) - offset;
	return distance;
}

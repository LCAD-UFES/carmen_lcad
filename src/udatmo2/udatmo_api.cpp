#include "udatmo_api.h"

#include "datmo.h"
#include "primitives.h"
#include "udatmo_interface.h"

using udatmo::Obstacle;
using udatmo::Obstacles;
using udatmo::clear;
using udatmo::resize;
using udatmo::distance;
using udatmo::getDATMO;


#define NUM_OBSTACLES 1


void carmen_udatmo_init(const carmen_robot_ackerman_config_t *robot_config)
{
	getDATMO().setup(*robot_config);
}


void carmen_udatmo_setup(int argc, char *argv[])
{
	getDATMO().setup(argc, argv);
}


static void carmen_udatmo_resize_moving_obstacles_message(carmen_udatmo_moving_obstacles_message *message, int size)
{
	if (message->num_obstacles == size)
		return;

	message->num_obstacles = size;
	resize(size, message->obstacles);
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
		entry.rddf_lane = obstacle.lane();
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
		carmen_udatmo_moving_obstacle &obstacle = message->obstacles[i];

		clear(obstacle);
		obstacle.rddf_index = -1;
		obstacle.rddf_lane = -1;
	}
}


carmen_udatmo_moving_obstacles_message *carmen_udatmo_get_moving_obstacles(void)
{
	static carmen_udatmo_moving_obstacles_message *message = NULL;
	if (message == NULL)
		message = carmen_udatmo_new_moving_obstacles_message(NUM_OBSTACLES);

	return message;
}


carmen_udatmo_moving_obstacles_message *carmen_udatmo_detect_moving_obstacles(void)
{
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_get_moving_obstacles();

	const Obstacles &obstacles = getDATMO().track();
	if (obstacles.size() > 0)
		carmen_udatmo_set_moving_obstacles_message(message, obstacles);
	else
		carmen_udatmo_reset_moving_obstacles_message(message);

	return message;
}


<<<<<<< HEAD
static carmen_udatmo_moving_obstacle *carmen_udatmo_find_front_moving_obstacle(carmen_udatmo_moving_obstacles_message *message)
=======
carmen_udatmo_moving_obstacle *carmen_udatmo_find_front_moving_obstacle(const carmen_udatmo_moving_obstacles_message *message)
>>>>>>> fbecc5bd0dd0d608b656ea796e3a54fe696aab19
{
	static carmen_udatmo_moving_obstacle dummy = {-1, -1, 0, 0, 0, 0};

	for (carmen_udatmo_moving_obstacle *o = message->obstacles, *n = o + message->num_obstacles; o != n; ++o)
		if (o->rddf_lane == 0)
			return o;

	return &dummy;
}


carmen_udatmo_moving_obstacle *carmen_udatmo_detect_front_moving_obstacle(void)
{
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_detect_moving_obstacles();
	return carmen_udatmo_find_front_moving_obstacle(message);
}


int carmen_udatmo_front_obstacle_detected(void)
{
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_get_moving_obstacles();
	carmen_udatmo_moving_obstacle* obstacle = carmen_udatmo_find_front_moving_obstacle(message);
	return (obstacle->rddf_index != -1);
}


double carmen_udatmo_front_obstacle_speed(carmen_ackerman_traj_point_t *robot_pose)
{
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_get_moving_obstacles();
	carmen_udatmo_moving_obstacle* obstacle = carmen_udatmo_find_front_moving_obstacle(message);

	// distance in the direction of the robot: https://en.wikipedia.org/wiki/Vector_projection
	return obstacle->v * cos(obstacle->theta - robot_pose->theta);
}


double carmen_udatmo_front_obstacle_distance(carmen_ackerman_traj_point_t *robot_pose)
{
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_get_moving_obstacles();
	carmen_udatmo_moving_obstacle* obstacle = carmen_udatmo_find_front_moving_obstacle(message);
	return distance(*robot_pose, *obstacle);
}


carmen_ackerman_traj_point_t carmen_udatmo_front_obstacle_position(void)
{
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_get_moving_obstacles();
	carmen_udatmo_moving_obstacle* obstacle = carmen_udatmo_find_front_moving_obstacle(message);
	carmen_ackerman_traj_point_t pose;
	pose.x = obstacle->x;
	pose.y = obstacle->y;
	pose.theta = obstacle->theta;
	pose.v = obstacle->v;
	pose.phi = 0;
	return pose;
}


void carmen_udatmo_update_robot_pose(const carmen_ackerman_traj_point_t *robot_pose)
{
	getDATMO().update(*robot_pose);
}



void carmen_udatmo_update_robot_pose_with_globalpos(carmen_localize_ackerman_globalpos_message *message)
{
	carmen_ackerman_traj_point_t robot_pose;
	robot_pose.x = message->globalpos.x;
	robot_pose.y = message->globalpos.y;
	robot_pose.theta = message->globalpos.theta;
	robot_pose.v = message->v;
	robot_pose.phi = message->phi;

	getDATMO().update(robot_pose);
}


void carmen_udatmo_update_robot_pose_with_truepos(carmen_simulator_ackerman_truepos_message *message)
{
	carmen_ackerman_traj_point_t robot_pose;
	robot_pose.x = message->truepose.x;
	robot_pose.y = message->truepose.y;
	robot_pose.theta = message->truepose.theta;
	robot_pose.v = message->v;
	robot_pose.phi = message->phi;

	getDATMO().update(robot_pose);
}


<<<<<<< HEAD
=======
void carmen_udatmo_update_distance_map(carmen_obstacle_distance_mapper_message *message)
{
	getDATMO().update(message);
}


>>>>>>> fbecc5bd0dd0d608b656ea796e3a54fe696aab19
void carmen_udatmo_update_offline_map(carmen_mapper_map_message *grid)
{
	getDATMO().update(grid);
}


void carmen_udatmo_update_rddf(carmen_rddf_road_profile_message *message)
{
	getDATMO().update(message);
}

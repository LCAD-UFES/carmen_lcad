#ifndef MESSAGECONTROL_HPP
#define MESSAGECONTROL_HPP

#include <carmen/carmen.h>
#include <assert.h>
#include "conventional_astar_ackerman.hpp"

#include "conventional_ackerman.h"
#include "navigator_astar.hpp"
#include "trajectory_ackerman.h"
#include "navigator_ackerman_ipc.h"

#include "path_planner_messages.h"


class MessageControl
{
private:
	carmen_map_t *carmen_planner_map;
	carmen_ackerman_traj_point_p requested_goal;
	carmen_robot_and_trailers_traj_point_t robot;
	carmen_robot_ackerman_config_t *robot_conf_g;
	carmen_planner_path_t path;
	AstarAckerman astarAckeman;
	carmen_obstacle_distance_mapper_map_message *distanceMap = NULL;


public:
	MessageControl()
	{
		carmen_planner_map = NULL;
		requested_goal = NULL;
		robot = {0, 0, 0, 0, 0, 0};
		robot_conf_g = NULL;
		path = {NULL, 0, 0};
	}

	void plan();
	void carmen_planner_ackerman_regenerate_trajectory();
	int carmen_planner_ackerman_update_robot(carmen_robot_and_trailers_traj_point_t *new_position, carmen_robot_ackerman_config_t *robot_conf);
	void carmen_planner_ackerman_set_cost_map(carmen_map_t *new_map);
	int carmen_planner_ackerman_setDistanceMap(carmen_obstacle_distance_mapper_map_message *newDistanceMap);

	int	carmen_planner_ackerman_update_goal(carmen_ackerman_traj_point_t *goal);
	void carmen_planner_ackerman_get_status(carmen_planner_status_p status);
	void initAstarParans(carmen_navigator_ackerman_astar_t astar_config);

};

#endif

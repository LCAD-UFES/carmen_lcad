#include "MessageControl.hpp"
#include "iostream"

#include "conventional_astar_ackerman.hpp"

void
MessageControl::plan()
{
	int goal_x, goal_y;

	goal_x = carmen_round((this->requested_goal->x - carmen_planner_map->config.x_origin) / carmen_planner_map->config.resolution);
	goal_y = carmen_round((this->requested_goal->y - carmen_planner_map->config.y_origin) / carmen_planner_map->config.resolution);

	carmen_conventional_dynamic_program(goal_x , goal_y);
}


void
MessageControl::carmen_planner_ackerman_regenerate_trajectory()
{
	static carmen_map_point_t map_pt;

	carmen_ackerman_trajectory_to_map(&robot, &map_pt, carmen_planner_map);
	if (carmen_conventional_get_utility(map_pt.x, map_pt.y) < 0)
		return;

	this->path.capacity = 20;
	this->path.points = (carmen_robot_and_trailers_traj_point_t *) calloc(path.capacity, sizeof(carmen_robot_and_trailers_traj_point_t));
	carmen_test_alloc(path.points);

	this->astarAckeman.carmen_planner_map =carmen_planner_map;
	this->astarAckeman.carmen_conventional_astar_ackerman_astar(robot, *requested_goal, &path, robot_conf_g);

}


int 
MessageControl::carmen_planner_ackerman_update_robot(carmen_robot_and_trailers_traj_point_t *new_position, carmen_robot_ackerman_config_t *robot_conf)
{
	static carmen_robot_and_trailers_traj_point_t lastPosition;
	static int first_time = 1;
	robot_conf_g = robot_conf;

	if (!carmen_planner_map)
		return false;

	if (!requested_goal)
		return false;

	if ((new_position->x - carmen_planner_map->config.x_origin) < 0 ||
	(new_position->y - carmen_planner_map->config.y_origin) < 0 ||
	(new_position->x - carmen_planner_map->config.x_origin) > carmen_planner_map->config.resolution * carmen_planner_map->config.x_size ||
	(new_position->y - carmen_planner_map->config.y_origin) > carmen_planner_map->config.resolution*carmen_planner_map->config.y_size)
		return 0;

	robot = *new_position;

	if (!first_time && carmen_distance_ackerman_traj(new_position, &lastPosition) < carmen_planner_map->config.resolution)
	{
		return false;
	}
	lastPosition = *new_position;

	carmen_planner_ackerman_regenerate_trajectory();

	return 1;
}

void
MessageControl::carmen_planner_ackerman_set_cost_map(carmen_map_t *new_map)
{
	carmen_planner_map = new_map;
	carmen_conventional_set_costs(new_map);

	if (this->requested_goal)
	{
		plan();
		carmen_planner_ackerman_regenerate_trajectory();
	}
}


int
MessageControl::carmen_planner_ackerman_update_goal(carmen_robot_and_trailers_traj_point_t *goal)
{
	if (!carmen_planner_map)
		return false;

	requested_goal = goal;

	plan();
	carmen_planner_ackerman_regenerate_trajectory();

	return (true);
}


void 
MessageControl::carmen_planner_ackerman_get_status(carmen_planner_status_p status)
{
	int index;
	if (!requested_goal)
		return;

	status->goal = *requested_goal;
	status->robot = robot;
	status->goal_set = (requested_goal == NULL);
	status->path.length = path.length;

	if (status->path.length > 0)
	{
		status->path.points = (carmen_robot_and_trailers_traj_point_t *) calloc(status->path.length, sizeof(carmen_robot_and_trailers_traj_point_t));
		carmen_test_alloc(status->path.points);
		for (index = 0; index < status->path.length; index++)
			status->path.points[index] = path.points[index];
	}
	else
	{
		status->path.length = 0;
	}
	return;
}

void MessageControl::initAstarParans(carmen_navigator_ackerman_astar_t astar_config)
{
	astarAckeman.astar_config = astar_config;
}

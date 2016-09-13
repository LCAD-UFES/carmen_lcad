/*
 * behavior_selector.c
 *
 *  Created on: 28/09/2012
 *      Author: romulo
 */

#include "behavior_selector.h"
#include "behavior_selector_messages.h"
#include <carmen/collision_detection.h>
#include <carmen/motion_planner_interface.h>

#define GOAL_LIST_SIZE 1000
#define MAX_ANNOTATIONS 50

static double goal_list_time = 0;
static carmen_robot_ackerman_config_t robot_config;
static double distance_between_waypoints = 5;
static carmen_ackerman_traj_point_t robot_pose;
static int robot_initialized = 0;
static carmen_ackerman_traj_point_t goal_list[GOAL_LIST_SIZE];
static int annotations[GOAL_LIST_SIZE];
static int goal_list_index = 0;
static int goal_list_size = 0;
static carmen_map_t *current_map = NULL;
static carmen_behavior_selector_state_t current_state = BEHAVIOR_SELECTOR_FOLLOWING_LANE;
static carmen_behavior_selector_goal_source_t current_goal_source = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
static double change_goal_distance = 8.0;
static carmen_behavior_selector_algorithm_t following_lane_planner;
static carmen_behavior_selector_algorithm_t parking_planner;

static carmen_rddf_road_profile_message *last_rddf_message = NULL;
int position_of_next_annotation = 0;
int num_poses_with_annotations = 0;
int poses_with_annotations[MAX_ANNOTATIONS];

// filipe:: TODO: colocar no carmen.ini
double dist_to_reduce_speed = 15.0;
double speed_around_annotation = 1.0;


static void
change_state(int rddf_annotation);


carmen_behavior_selector_algorithm_t
get_current_algorithm()
{
	carmen_behavior_selector_algorithm_t current_algorithm = CARMEN_BEHAVIOR_SELECTOR_INVALID_PLANNER;

	switch(current_state)
	{
	case BEHAVIOR_SELECTOR_FOLLOWING_LANE:
		current_algorithm = following_lane_planner;
		break;
	case BEHAVIOR_SELECTOR_PARKING:
		current_algorithm = parking_planner;
		break;
	default:
		current_algorithm = following_lane_planner;
		break;
	}

	return current_algorithm;
}


static void
copy_rddf_message(carmen_rddf_road_profile_message *rddf_msg)
{
	if (!last_rddf_message)
	{
		last_rddf_message = (carmen_rddf_road_profile_message*)malloc(sizeof(carmen_rddf_road_profile_message));
		last_rddf_message->number_of_poses = 0;
		last_rddf_message->poses = NULL;
		last_rddf_message->annotations = NULL;
	}

	if (last_rddf_message->number_of_poses != rddf_msg->number_of_poses)
	{
		last_rddf_message->number_of_poses = rddf_msg->number_of_poses;

		free(last_rddf_message->poses);
		free(last_rddf_message->annotations);

		last_rddf_message->poses = (carmen_ackerman_traj_point_t*) malloc(sizeof(carmen_ackerman_traj_point_t) * last_rddf_message->number_of_poses);
		last_rddf_message->annotations = (int*) malloc(sizeof(int) * last_rddf_message->number_of_poses);
	}

	last_rddf_message->timestamp = rddf_msg->timestamp;

	memcpy(last_rddf_message->poses, rddf_msg->poses, sizeof(carmen_ackerman_traj_point_t) * last_rddf_message->number_of_poses);
	memcpy(last_rddf_message->annotations, rddf_msg->annotations, sizeof(int) * last_rddf_message->number_of_poses);
}


static void
get_next_goal(carmen_rddf_road_profile_message *rddf, carmen_ackerman_traj_point_t current_pose, carmen_ackerman_traj_point_t *next_goal)
{
	double distance_to_last_obstacle = 10000.0;
	int last_obstacle_index = -1;

	for (int i = 0; i < rddf->number_of_poses; i++)
	{
		double distance = carmen_distance_ackerman_traj(&current_pose, &rddf->poses[i]);
		int hit_obstacle = trajectory_pose_hit_obstacle(rddf->poses[i], current_map, &robot_config);
		if (hit_obstacle)
			last_obstacle_index = i;

		if (last_obstacle_index != -1)
			distance_to_last_obstacle = carmen_distance_ackerman_traj(&rddf->poses[last_obstacle_index], &rddf->poses[i]);

		if (((distance >= distance_between_waypoints) &&
			 (distance_to_last_obstacle >= 8.0) &&
			 !hit_obstacle) ||
			 ((rddf->annotations[i] == RDDF_ANNOTATION_TYPE_BUMP) || (rddf->annotations[i] == RDDF_ANNOTATION_TYPE_BARRIER)))
		{
			*next_goal = rddf->poses[i];
			return;
		}
	}

	next_goal->x = 0;
	next_goal->y = 0;

	return;
}


static void
fill_goal_list(carmen_rddf_road_profile_message *rddf, carmen_ackerman_traj_point_t current_pose)
{
	double distance_to_last_obstacle = 10000.0;
	int last_obstacle_index = -1;
	int j = 0;

	int last_obstacle_free_waypoint_index = -1;
	carmen_ackerman_traj_point_t robot_pose = current_pose;

	for (int i = 0; i < rddf->number_of_poses && j < GOAL_LIST_SIZE; i++)
	{
		double distance = carmen_distance_ackerman_traj(&current_pose, &rddf->poses[i]);
		int hit_obstacle = trajectory_pose_hit_obstacle(rddf->poses[i], current_map, &robot_config);

		if (hit_obstacle)
			last_obstacle_index = i;
		else
			last_obstacle_free_waypoint_index = i;

		if (last_obstacle_index != -1)
			distance_to_last_obstacle = carmen_distance_ackerman_traj(&rddf->poses[last_obstacle_index], &rddf->poses[i]);

		double distance_to_annotation = carmen_distance_ackerman_traj(&robot_pose, &rddf->poses[i]);
		double distance_to_last_obstacle_free_waypoint = carmen_distance_ackerman_traj(&robot_pose, &rddf->poses[last_obstacle_free_waypoint_index]);

		if (((distance >= distance_between_waypoints) &&
			 (distance_to_last_obstacle >= 15.0) &&
			 !hit_obstacle) ||
			(((rddf->annotations[i] == RDDF_ANNOTATION_TYPE_BUMP) ||
			  (rddf->annotations[i] == RDDF_ANNOTATION_TYPE_BARRIER)) &&
			 (distance_to_annotation > 2.0) && (!hit_obstacle)))
		{
			goal_list[j] = rddf->poses[i];
			annotations[j] = rddf->annotations[i];
			current_pose = rddf->poses[i];
			j++;
		}
		// se a anotacao estiver em cima de um obstaculo, adiciona um waypoint na posicao
		// anterior mais proxima da anotacao que estiver livre
		else if (((rddf->annotations[i] == RDDF_ANNOTATION_TYPE_BUMP) || (rddf->annotations[i] == RDDF_ANNOTATION_TYPE_BARRIER)) && (distance_to_last_obstacle_free_waypoint > 1.5) && (hit_obstacle))
		{
			goal_list[j] = rddf->poses[last_obstacle_free_waypoint_index];
			annotations[j] = rddf->annotations[last_obstacle_free_waypoint_index];
			current_pose = rddf->poses[last_obstacle_free_waypoint_index];
			j++;
		}
	}

	goal_list_size = j;
	goal_list_time = rddf->timestamp;

	if (goal_list_index < goal_list_size)
		change_state(annotations[goal_list_index]);
}


static void
update_goal_index_to_next_collision_free_goal()
{
	carmen_ackerman_traj_point_t next_goal;
	if (last_rddf_message != NULL /*&& trajectory_pose_hit_obstacle(goal_list[goal_list_index], current_map, &robot_config)*/)
	{
		get_next_goal(last_rddf_message, robot_pose, &next_goal);

		if ((goal_list_index < goal_list_size) &&
			(carmen_distance_ackerman_traj(&robot_pose, &next_goal) < carmen_distance_ackerman_traj(&robot_pose, &goal_list[goal_list_index])))
		{
			goal_list_index = 0;
			goal_list_size = 0;
			fill_goal_list(last_rddf_message, robot_pose);
		}
	}
}


static int
change_goal()
{
	double distance_to_goal;

	if (goal_list_index >= goal_list_size)
	{
		goal_list_size = 0;
		goal_list_index = 0;
		return 0;
	}

	//ultimo ponto do rddf
	if (goal_list_size == 1)
	{
		return 0;
	}

	distance_to_goal = carmen_distance_ackerman_traj(&robot_pose, &goal_list[goal_list_index]);

	if (distance_to_goal < change_goal_distance)
	{
		if (current_goal_source != CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
		{
			goal_list_index = 0;
			goal_list_size = 0;
			fill_goal_list(last_rddf_message, robot_pose);
		}
		else if (goal_list_index != (goal_list_size - 1) || current_state != BEHAVIOR_SELECTOR_PARKING)
		{
			goal_list_index++;
		}
		return 1;
	}

	return 0;
}


int
behavior_selector_set_state(carmen_behavior_selector_state_t state)
{
	if (state == current_state)
		return (0);

	current_state = state;

	return (1);
}


void
behavior_selector_get_state(carmen_behavior_selector_state_t *current_state_out, carmen_behavior_selector_algorithm_t *following_lane_planner_out,
		carmen_behavior_selector_algorithm_t *parking_planner_out, carmen_behavior_selector_goal_source_t *current_goal_source_out)
{
	*current_state_out = current_state;
	*following_lane_planner_out = following_lane_planner;
	*parking_planner_out = parking_planner;
	*current_goal_source_out = current_goal_source;
}


void
behavior_selector_get_goal_list(carmen_ackerman_traj_point_t **goal_list_out, int *goal_list_size_out, int *goal_list_index_out, double *goal_list_time_out)
{
	*goal_list_out = goal_list;
	*goal_list_size_out = goal_list_size;
	*goal_list_index_out = goal_list_index;

	if (goal_list_time == 0)
		*goal_list_time_out = carmen_get_time();
	else
		*goal_list_time_out = goal_list_time;
}


int
behavior_selector_set_goal_source(carmen_behavior_selector_goal_source_t goal_source)
{
	if (current_goal_source == goal_source)
		return (0);

	current_goal_source = goal_source;

	goal_list_index = 0;
	goal_list_size = 0;

	return (1);
}


void
behavior_selector_add_goal(carmen_point_t goal)
{
	goal_list[goal_list_size].x = goal.x;
	goal_list[goal_list_size].y = goal.y;
	goal_list[goal_list_size].theta = goal.theta;
	goal_list_size++;
}


void
behavior_selector_clear_goal_list()
{
	goal_list_index = 0;
	goal_list_size = 0;
}

void behavior_selector_remove_goal()
{
	goal_list_size--;
	if (goal_list_size < 0)
		goal_list_size = 0;
}

void
behavior_selector_set_algorithm(carmen_behavior_selector_algorithm_t algorithm, carmen_behavior_selector_state_t state)
{
	switch(state)
	{
	case BEHAVIOR_SELECTOR_FOLLOWING_LANE:
		following_lane_planner = algorithm;
		break;

	case BEHAVIOR_SELECTOR_PARKING:
		parking_planner = algorithm;
		break;
	default:
		following_lane_planner = algorithm;
		break;
	}
}


static void
change_state(int rddf_annotation)
{
	if (current_goal_source == CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
		return;

	switch(rddf_annotation)
	{
	case RDDF_ANNOTATION_NONE:
		current_state = BEHAVIOR_SELECTOR_FOLLOWING_LANE;
		break;

	case RDDF_ANNOTATION_END_POINT_AREA:
		current_state = BEHAVIOR_SELECTOR_PARKING;
		break;

	case RDDF_ANNOTATION_HUMAN_INTERVENTION:
		current_state = BEHAVIOR_SELECTOR_HUMAN_INTERVENTION;
		carmen_navigator_ackerman_stop();
		break;
	}
}


void
behavior_selector_update_robot_pose(carmen_ackerman_traj_point_t pose, int *state_updated)
{
	if (carmen_distance_ackerman_traj(&robot_pose, &pose) > 2.5 && current_goal_source != CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
	{
		//provavelmente o robo foi reposicionado
		goal_list_size = 0;
		goal_list_index = 0;
	}

	robot_pose = pose;
	robot_initialized = 1;

	if (change_goal() && goal_list_index < goal_list_size)
	{
		change_state(annotations[goal_list_index]);
		*state_updated = 1;
	}
	else
		*state_updated = 0;
}


carmen_ackerman_traj_point_t
get_robot_pose()
{
	return (robot_pose);
}


double
get_max_v()
{
	return (robot_config.max_v);
}


void
behavior_selector_update_rddf(carmen_rddf_road_profile_message *rddf_msg)
{
	copy_rddf_message(rddf_msg);

	if ((get_current_algorithm() == CARMEN_BEHAVIOR_SELECTOR_RDDF) && (last_rddf_message) && (last_rddf_message->number_of_poses > 0))
		carmen_motion_planner_publish_path_message(last_rddf_message->poses, last_rddf_message->number_of_poses, CARMEN_BEHAVIOR_SELECTOR_RDDF);

	if ((!robot_initialized) || (current_goal_source == CARMEN_BEHAVIOR_SELECTOR_USER_GOAL))
		return;

	if (((goal_list_size - goal_list_index) > 1) &&
		(rddf_msg->annotations[rddf_msg->number_of_poses - 1] != RDDF_ANNOTATION_END_POINT_AREA || (rddf_msg->annotations[rddf_msg->number_of_poses - 1] == RDDF_ANNOTATION_END_POINT_AREA &&
		carmen_distance_ackerman_traj(&goal_list[goal_list_size - 1], &rddf_msg->poses[rddf_msg->number_of_poses - 1]) < 0.1)))
	{
		//return;
	}

	goal_list_index = 0;
	goal_list_size = 0;

	fill_goal_list(rddf_msg, robot_pose);
	change_goal();
}


void
behavior_selector_update_map(carmen_map_t *map, int *goal_list_updated)
{
	if (current_map)
		carmen_map_destroy(&current_map);

	current_map = carmen_map_clone(map);

	if (current_goal_source != CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
	{
		update_goal_index_to_next_collision_free_goal();
		*goal_list_updated = 1;
	}
	else
		*goal_list_updated = 0;
}


void
change_distance_between_waypoints_and_goals(double dist_between_waypoints, double change_goal_dist)
{
	distance_between_waypoints = dist_between_waypoints;
	change_goal_distance = change_goal_dist;
}


void
behavior_selector_initialize(carmen_robot_ackerman_config_t config, double dist_between_waypoints, double change_goal_dist, carmen_behavior_selector_algorithm_t f_planner, carmen_behavior_selector_algorithm_t p_planner)
{
	robot_config = config;
	distance_between_waypoints = dist_between_waypoints;
	change_goal_distance = change_goal_dist;
	parking_planner = p_planner;
	following_lane_planner = f_planner;
}


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
static double distance_to_remove_annotation_goal = 3.0;

static carmen_rddf_road_profile_message *last_rddf_message = NULL;
int position_of_next_annotation = 0;
int num_poses_with_annotations = 0;
int poses_with_annotations[MAX_ANNOTATIONS];

int carmen_rddf_num_poses_ahead = 100;
int carmen_rddf_num_poses_ahead_min = 40;
int carmen_rddf_num_poses_ahead_limited_by_map = 150;


// filipe:: TODO: colocar no carmen.ini
double dist_to_reduce_speed = 15.0;
double speed_around_annotation = 1.0;

MOVING_OBJECT moving_object[MOVING_OBJECT_HISTORY_SIZE];


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


int
compute_max_rddf_num_poses_ahead(carmen_ackerman_traj_point_t current_pose)
{
	int num_poses_ahead_by_velocity = carmen_rddf_num_poses_ahead_min;
	//	s = vf*vf - v0*v0 / 2*a;
	double common_goal_v = 3.0;
	double a = 0.0;
	double distance = 0.0;

	if (common_goal_v < current_pose.v)
	{
		a = -robot_config.maximum_acceleration_forward * 2.0;
		distance = ((common_goal_v * common_goal_v) - (current_pose.v * current_pose.v))/(2 * a);
		if (distance > 0)
			num_poses_ahead_by_velocity = (distance / 0.5) + 1;
	}
	if (num_poses_ahead_by_velocity < carmen_rddf_num_poses_ahead_min)
		num_poses_ahead_by_velocity = carmen_rddf_num_poses_ahead_min;
	else if (num_poses_ahead_by_velocity > carmen_rddf_num_poses_ahead_limited_by_map)
		num_poses_ahead_by_velocity = carmen_rddf_num_poses_ahead_limited_by_map;

//	printf("\n current_v: %lf distance: %lf a: %lf num_poses: %d \n", current_pose.v, distance, a, num_poses_ahead_by_velocity);
	return num_poses_ahead_by_velocity;
}


static void
copy_rddf_message(carmen_rddf_road_profile_message *rddf_msg)
{
	//Now the rddf is number of posses variable with the velocity
	if (!last_rddf_message)
	{
		last_rddf_message = (carmen_rddf_road_profile_message*)malloc(sizeof(carmen_rddf_road_profile_message));
		last_rddf_message->number_of_poses = 0;
		last_rddf_message->poses = NULL;
		last_rddf_message->annotations = NULL;
		last_rddf_message->poses_back = NULL;
		last_rddf_message->number_of_poses_back = 0;
	}

	 carmen_rddf_num_poses_ahead = compute_max_rddf_num_poses_ahead(robot_pose);

	if (last_rddf_message->number_of_poses != carmen_rddf_num_poses_ahead)
	{
		last_rddf_message->number_of_poses = carmen_rddf_num_poses_ahead;

		free(last_rddf_message->poses);
		free(last_rddf_message->annotations);
		free(last_rddf_message->poses_back);

		last_rddf_message->poses = (carmen_ackerman_traj_point_t*) malloc(sizeof(carmen_ackerman_traj_point_t) * last_rddf_message->number_of_poses);
		last_rddf_message->annotations = (int*) malloc(sizeof(int) * last_rddf_message->number_of_poses);

		if (rddf_msg->number_of_poses_back > 0)
		{
			last_rddf_message->number_of_poses_back = carmen_rddf_num_poses_ahead;
			last_rddf_message->poses_back = (carmen_ackerman_traj_point_t*) malloc(sizeof(carmen_ackerman_traj_point_t) * last_rddf_message->number_of_poses_back);
		}
	}

	last_rddf_message->timestamp = rddf_msg->timestamp;
	for (int i = 0; i < carmen_rddf_num_poses_ahead; i++)
	{
		last_rddf_message->poses[i] = rddf_msg->poses[i];
		last_rddf_message->annotations[i] = rddf_msg->annotations[i];
	}
	for (int i = 0; i < last_rddf_message->number_of_poses_back; i++)
		last_rddf_message->poses_back[i] = rddf_msg->poses_back[i];
}


static void
copy_rddf_message_old(carmen_rddf_road_profile_message *rddf_msg)
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


//static void
//get_next_goal(carmen_rddf_road_profile_message *rddf, carmen_ackerman_traj_point_t current_pose, carmen_ackerman_traj_point_t *next_goal)
//{
//	double distance_to_last_obstacle = 10000.0;
//	int last_obstacle_index = -1;
//
//	for (int i = 0; i < rddf->number_of_poses; i++)
//	{
//		double distance = carmen_distance_ackerman_traj(&current_pose, &rddf->poses[i]);
//		int hit_obstacle = trajectory_pose_hit_obstacle(rddf->poses[i], current_map, &robot_config);
//		if (hit_obstacle)
//			last_obstacle_index = i;
//
//		if (last_obstacle_index != -1)
//			distance_to_last_obstacle = carmen_distance_ackerman_traj(&rddf->poses[last_obstacle_index], &rddf->poses[i]);
//
//		if (((distance >= distance_between_waypoints) &&
//			 (distance_to_last_obstacle >= 8.0) &&
//			 !hit_obstacle) ||
//			 ((rddf->annotations[i] == RDDF_ANNOTATION_TYPE_BUMP) || (rddf->annotations[i] == RDDF_ANNOTATION_TYPE_BARRIER)))
//		{
//			*next_goal = rddf->poses[i];
//			return;
//		}
//	}
//
//	next_goal->x = 0;
//	next_goal->y = 0;
//
//	return;
//}


void
shift_moving_object_history()
{
	for (int i = MOVING_OBJECT_HISTORY_SIZE - 1; i > 0 ; i--)
		moving_object[i] = moving_object[i - 1];
}


void
update_moving_object_velocity()
{
	for (int i = MOVING_OBJECT_HISTORY_SIZE - 2; i >= 0 ; i--)
	{
		double distance = carmen_distance_ackerman_traj(&moving_object[i].pose, &moving_object[i + 1].pose);
		double delta_t = moving_object[i].timestamp - moving_object[i + 1].timestamp;
		double v = 0.0;
		if (delta_t > 0.01)
			v = distance / delta_t;
		moving_object[i].pose.v = v;
	}
}


int
get_moving_object_in_front_index()
{
	for (int i = 0; i < MOVING_OBJECT_HISTORY_SIZE ; i++)
	{
		if ((moving_object[i].pose.v > 0.2) && (moving_object[i].index == 0))
			return (0);
	}
	return (-1);
}


static int
detect_moving_object_in_front(carmen_rddf_road_profile_message *rddf, int goal_index, int rddf_pose_index, double timestamp)
{
	shift_moving_object_history();

	moving_object[0].valid = true;
	moving_object[0].pose = rddf->poses[rddf_pose_index];
	moving_object[0].index = goal_index;
	moving_object[0].timestamp = timestamp;

	update_moving_object_velocity();

	int moving_object_in_front_index = get_moving_object_in_front_index();

	return (moving_object_in_front_index);
}


static void
fill_goal_list(carmen_rddf_road_profile_message *rddf, carmen_ackerman_traj_point_t current_pose, double timestamp)
{
	double distance_to_last_obstacle = 10000.0;
	int last_obstacle_index = -1;

	int last_obstacle_free_waypoint_index = -1;
	carmen_ackerman_traj_point_t robot_pose = current_pose;

	if (rddf == NULL)
		return;

	int goal_index = 0;
	for (int rddf_pose_index = 0; rddf_pose_index < rddf->number_of_poses && goal_index < GOAL_LIST_SIZE; rddf_pose_index++)
	{
		double distance_from_car_to_rddf_point = carmen_distance_ackerman_traj(&current_pose, &rddf->poses[rddf_pose_index]);
		int rddf_pose_hit_obstacle = trajectory_pose_hit_obstacle(rddf->poses[rddf_pose_index], current_map, &robot_config);

		int moving_object_in_front_index = -1;
		if (rddf_pose_hit_obstacle)
		{
			moving_object_in_front_index = detect_moving_object_in_front(rddf, goal_index, rddf_pose_index, timestamp);
			last_obstacle_index = rddf_pose_index;
		}
		else
			last_obstacle_free_waypoint_index = rddf_pose_index;

		if (last_obstacle_index != -1)
			distance_to_last_obstacle = carmen_distance_ackerman_traj(&rddf->poses[last_obstacle_index], &rddf->poses[rddf_pose_index]);

		double distance_to_annotation = carmen_distance_ackerman_traj(&robot_pose, &rddf->poses[rddf_pose_index]);
		double distance_to_last_obstacle_free_waypoint = carmen_distance_ackerman_traj(&robot_pose, &rddf->poses[last_obstacle_free_waypoint_index]);

		if (rddf_pose_index == moving_object_in_front_index)
		{
			goal_list[goal_index] = rddf->poses[rddf_pose_index];
			annotations[goal_index] = rddf->annotations[rddf_pose_index];
			current_pose = rddf->poses[rddf_pose_index];
			goal_index++;
		}
		else if (((distance_from_car_to_rddf_point >= distance_between_waypoints) &&
			 (distance_to_last_obstacle >= 15.0) &&
			 !rddf_pose_hit_obstacle) ||
			(((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BUMP) ||
			  (rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BARRIER) ||
			  (rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK)) &&
			 (distance_to_annotation > distance_to_remove_annotation_goal) && (!rddf_pose_hit_obstacle)))
		{
			goal_list[goal_index] = rddf->poses[rddf_pose_index];
			annotations[goal_index] = rddf->annotations[rddf_pose_index];
			current_pose = rddf->poses[rddf_pose_index];
			goal_index++;
		}
		else if (((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BUMP) || (rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BARRIER)) &&
				 (distance_to_last_obstacle_free_waypoint > 1.5) && rddf_pose_hit_obstacle)
		{	// se a anotacao estiver em cima de um obstaculo, adiciona um waypoint na posicao anterior mais proxima da anotacao que estiver livre
			goal_list[goal_index] = rddf->poses[last_obstacle_free_waypoint_index];
			annotations[goal_index] = rddf->annotations[last_obstacle_free_waypoint_index];
			current_pose = rddf->poses[last_obstacle_free_waypoint_index];
			goal_index++;
		}
		else if ((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK) && rddf_pose_hit_obstacle)
		{
			goal_list[goal_index] = rddf->poses[last_obstacle_free_waypoint_index];
			annotations[goal_index] = rddf->annotations[last_obstacle_free_waypoint_index];
			current_pose = rddf->poses[last_obstacle_free_waypoint_index];
			goal_index++;

			break;
		}
	}

	if (goal_index == 0)
	{
		goal_list[goal_index] = rddf->poses[rddf->number_of_poses - 1];
		goal_index++;
	}

	goal_list_size = goal_index;
	if (goal_list_index < goal_list_size)
		change_state(annotations[goal_list_index]);
}


//static void
//update_goal_index_to_next_collision_free_goal(double timestamp)
//{
//	carmen_ackerman_traj_point_t next_goal;
//	if (last_rddf_message != NULL /*&& trajectory_pose_hit_obstacle(goal_list[goal_list_index], current_map, &robot_config)*/)
//	{
//		get_next_goal(last_rddf_message, robot_pose, &next_goal);
//
//		if ((goal_list_index < goal_list_size) &&
//			(carmen_distance_ackerman_traj(&robot_pose, &next_goal) < carmen_distance_ackerman_traj(&robot_pose, &goal_list[goal_list_index])))
//		{
//			goal_list_index = 0;
//			goal_list_size = 0;
//			fill_goal_list(last_rddf_message, robot_pose, timestamp);
//		}
//	}
//}


static int
change_goal_list(double timestamp)
{
//	double distance_to_goal;
//
//	if (goal_list_index >= goal_list_size)
//	{
//		goal_list_size = 0;
//		goal_list_index = 0;
//
//		return 0;
//	}
//
//	// ultimo ponto do rddf
//	if (goal_list_size == 1)
//		return 0;
//
//	distance_to_goal = carmen_distance_ackerman_traj(&robot_pose, &goal_list[goal_list_index]);
//
//	if (distance_to_goal < change_goal_distance)
//	{
//		if (current_goal_source != CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
//		{
			goal_list_index = 0;
			goal_list_size = 0;
			fill_goal_list(last_rddf_message, robot_pose, timestamp);
//		}
//		else if (goal_list_index != (goal_list_size - 1) || current_state != BEHAVIOR_SELECTOR_PARKING)
//		{
//			goal_list_index++;
//		}
		return 1;
//	}
//
//	return 0;
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


carmen_ackerman_traj_point_t *
behavior_selector_get_goal_list(int *goal_list_size_out)
{
	*goal_list_size_out = goal_list_size;
	*goal_list_size_out -= goal_list_index;

	carmen_ackerman_traj_point_t *goal_list_out = goal_list;
	goal_list_out += goal_list_index;

	return (goal_list_out);
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


void
behavior_selector_remove_goal()
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


int
update_goal_list(double timestamp)
{
	if (change_goal_list(timestamp) && goal_list_index < goal_list_size)
	{
		change_state(annotations[goal_list_index]);
		return (1);
	}
	else
		return (0);
}


void
behavior_selector_update_robot_pose(carmen_ackerman_traj_point_t pose)
{
	if (carmen_distance_ackerman_traj(&robot_pose, &pose) > 2.5 && current_goal_source != CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
	{
		//provavelmente o robo foi reposicionado
		goal_list_size = 0;
		goal_list_index = 0;
	}

	robot_pose = pose;
	robot_initialized = 1;
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


carmen_robot_ackerman_config_t *
get_robot_config()
{
	return (&robot_config);
}


carmen_rddf_road_profile_message *
get_last_rddf_message()
{
	return last_rddf_message;
}


void
behavior_selector_update_rddf(carmen_rddf_road_profile_message *rddf_msg, int rddf_num_poses_by_velocity, double timestamp)
{
	if (rddf_num_poses_by_velocity)
		copy_rddf_message(rddf_msg);
	else
		copy_rddf_message_old(rddf_msg);

	if ((get_current_algorithm() == CARMEN_BEHAVIOR_SELECTOR_RDDF) && (last_rddf_message) && (last_rddf_message->number_of_poses > 0))
		carmen_motion_planner_publish_path_message(last_rddf_message->poses, last_rddf_message->number_of_poses, CARMEN_BEHAVIOR_SELECTOR_RDDF);

	if ((!robot_initialized) || (current_goal_source == CARMEN_BEHAVIOR_SELECTOR_USER_GOAL))
		return;

//	if (((goal_list_size - goal_list_index) > 1) &&
//		(rddf_msg->annotations[rddf_msg->number_of_poses - 1] != RDDF_ANNOTATION_END_POINT_AREA || (rddf_msg->annotations[rddf_msg->number_of_poses - 1] == RDDF_ANNOTATION_END_POINT_AREA &&
//		carmen_distance_ackerman_traj(&goal_list[goal_list_size - 1], &rddf_msg->poses[rddf_msg->number_of_poses - 1]) < 0.1)))
//	{
//		//return;
//	}

//	goal_list_index = 0;
//	goal_list_size = 0;

//	fill_goal_list(last_rddf_message, robot_pose, timestamp);
//	change_goal_list(timestamp);
}


void
behavior_selector_update_map(carmen_map_t *map)
{
	if (current_map)
		carmen_map_destroy(&current_map);

	current_map = carmen_map_clone(map);

//	if (current_goal_source != CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
//		update_goal_index_to_next_collision_free_goal(timestamp);
}


void
change_distance_between_waypoints_and_goals(double dist_between_waypoints, double change_goal_dist)
{
	distance_between_waypoints = dist_between_waypoints;
	change_goal_distance = change_goal_dist;
}


void
behavior_selector_initialize(carmen_robot_ackerman_config_t config, double dist_between_waypoints, double change_goal_dist,
		carmen_behavior_selector_algorithm_t f_planner, carmen_behavior_selector_algorithm_t p_planner,
		double dist_to_remove_annotation_goal, int rddf_num_poses_ahead_min, int rddf_num_poses_ahead_limited_by_map)
{
	robot_config = config;
	distance_between_waypoints = dist_between_waypoints;
	change_goal_distance = change_goal_dist;
	parking_planner = p_planner;
	following_lane_planner = f_planner;
	distance_to_remove_annotation_goal = dist_to_remove_annotation_goal;
	carmen_rddf_num_poses_ahead_min = rddf_num_poses_ahead_min;
	carmen_rddf_num_poses_ahead_limited_by_map = rddf_num_poses_ahead_limited_by_map;
	memset(moving_object, 0, sizeof(MOVING_OBJECT) * MOVING_OBJECT_HISTORY_SIZE);
}

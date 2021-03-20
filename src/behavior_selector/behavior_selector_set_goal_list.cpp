/*
 *
 *  Created on: 20/04/2020
 *      Author: Alberto
 */

#include <carmen/collision_detection.h>
#include <carmen/udatmo.h>
#include <carmen/global_graphics.h>
#include "behavior_selector.h"
#include "behavior_selector_messages.h"

#define GOAL_LIST_SIZE 1000
#define MAX_ANNOTATIONS 50

//#define PRINT_UDATMO_LOG

//uncomment return 0 to enable overtaking
//#define OVERTAKING

carmen_robot_ackerman_config_t robot_config;
static double robot_max_velocity_reverse;
static double distance_between_waypoints = 5;
static carmen_ackerman_traj_point_t robot_pose;
static int robot_initialized = 0;
static carmen_ackerman_traj_point_t goal_list[GOAL_LIST_SIZE];
static int goal_type[GOAL_LIST_SIZE];
static int annotations[GOAL_LIST_SIZE];
static int goal_list_size = 0;
static carmen_obstacle_distance_mapper_map_message *current_map = NULL;
//static carmen_behavior_selector_state_t current_state = BEHAVIOR_SELECTOR_PARK;
static carmen_behavior_selector_task_t current_task = BEHAVIOR_SELECTOR_FOLLOW_ROUTE;
static carmen_behavior_selector_goal_source_t current_goal_source = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
static double change_goal_distance = 8.0; // @@@ Alberto: acho que nao usa... deletar?
static carmen_behavior_selector_algorithm_t following_lane_planner;
static carmen_behavior_selector_algorithm_t parking_planner;
static double distance_to_remove_annotation_goal = 1.5;

extern int behavior_selector_reverse_driving;

int position_of_next_annotation = 0;


// filipe:: TODO: colocar no carmen.ini
double dist_to_reduce_speed = 15.0;
double speed_around_annotation = 1.0;

//MOVING_OBJECT moving_object[MOVING_OBJECT_HISTORY_SIZE];
//int moving_object_in_front_detected = 0;

#define MAX_VIRTUAL_LASER_SAMPLES 1000000
extern carmen_mapper_virtual_laser_message virtual_laser_message;

//SampleFilter filter;
SampleFilter filter2;

extern bool wait_start_moving;

extern double map_width;

extern int behavior_selector_use_symotha;


carmen_behavior_selector_algorithm_t
get_current_algorithm()
{
	carmen_behavior_selector_algorithm_t current_algorithm = CARMEN_BEHAVIOR_SELECTOR_INVALID_PLANNER;

	switch(current_task)
	{
	case BEHAVIOR_SELECTOR_FOLLOW_ROUTE:
		current_algorithm = following_lane_planner;
		break;
	case BEHAVIOR_SELECTOR_PARK:
		current_algorithm = parking_planner;
		break;
	default:
		current_algorithm = following_lane_planner;
		break;
	}

	return current_algorithm;
}


void
change_task(int rddf_annotation)
{
	if (current_goal_source == CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
		return;

	switch(rddf_annotation)
	{
	case RDDF_ANNOTATION_TYPE_NONE:
		current_task = BEHAVIOR_SELECTOR_FOLLOW_ROUTE;
		break;

	case RDDF_ANNOTATION_TYPE_END_POINT_AREA:
		current_task = BEHAVIOR_SELECTOR_PARK;
		break;

	case RDDF_ANNOTATION_TYPE_HUMAN_INTERVENTION:
		current_task = BEHAVIOR_SELECTOR_HUMAN_INTERVENTION;
		carmen_navigator_ackerman_stop();
		break;
	}
}


void
add_goal_to_goal_list(int &goal_index, carmen_ackerman_traj_point_t &current_goal, int &current_goal_rddf_index, int rddf_pose_index,
		carmen_rddf_road_profile_message *rddf)
{
	goal_list[goal_index] = rddf->poses[rddf_pose_index];
	
//	if (carmen_sign(rddf->poses[rddf_pose_index].v) == 1)
//	{
//		goal_list[goal_index].v = get_max_v();
//	}
//	else
//	{
//		goal_list[goal_index].v = get_max_v_reverse();
//	}
	
	annotations[goal_index] = rddf->annotations[rddf_pose_index];
	current_goal = rddf->poses[rddf_pose_index];
	current_goal_rddf_index = rddf_pose_index;
	goal_index++;
}


void
add_goal_to_goal_list(int &goal_index, carmen_ackerman_traj_point_t &current_goal, int &current_goal_rddf_index, int rddf_pose_index,
		carmen_rddf_road_profile_message *rddf, double displacement)
{
	carmen_ackerman_traj_point_t new_car_traj_point = rddf->poses[rddf_pose_index];
	carmen_point_t new_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&new_car_traj_point, displacement);
	new_car_traj_point.x = new_car_pose.x;
	new_car_traj_point.y = new_car_pose.y;
	
//	if (carmen_sign(rddf->poses[rddf_pose_index].v) == 1)
//	{
//		new_car_traj_point.v = get_max_v();
//	}
//	else
//	{
//		new_car_traj_point.v = get_max_v_reverse();
//	}

	goal_list[goal_index] = new_car_traj_point;
	annotations[goal_index] = rddf->annotations[rddf_pose_index];
	current_goal = new_car_traj_point;
	current_goal_rddf_index = rddf_pose_index;
	goal_index++;
}


int
try_avoiding_obstacle(int rddf_pose_index, double circle_radius, carmen_rddf_road_profile_message* rddf)
{
	int rddf_pose_hit_obstacle = trajectory_pose_hit_obstacle(rddf->poses[rddf_pose_index], circle_radius, current_map, &robot_config);

#ifdef OVERTAKING
	return 0;
#endif
	return (rddf_pose_hit_obstacle);

//	if (rddf_pose_hit_obstacle == 1)
//	{
//		// try moving goal left
//		for (double displacement = 0.0; displacement < circle_radius; displacement += 0.2)
//		{
//			carmen_ackerman_traj_point_t displaced_rddf_pose = rddf->poses[rddf_pose_index];
//			displaced_rddf_pose.x = displaced_rddf_pose.x + displacement * cos(displaced_rddf_pose.theta + M_PI / 2.0);
//			displaced_rddf_pose.y = displaced_rddf_pose.y + displacement * sin(displaced_rddf_pose.theta + M_PI / 2.0);
//			if (!trajectory_pose_hit_obstacle(displaced_rddf_pose, circle_radius, current_map, &robot_config))
//			{
//				rddf->poses[rddf_pose_index] = displaced_rddf_pose;
//				rddf_pose_hit_obstacle = 0;
//				break;
//			}
//		}
//		if (rddf_pose_hit_obstacle)
//		{
//			// try moving goal right
//			for (double displacement = 0.0; displacement < circle_radius; displacement += 0.2)
//			{
//				carmen_ackerman_traj_point_t displaced_rddf_pose = rddf->poses[rddf_pose_index];
//				displaced_rddf_pose.x = displaced_rddf_pose.x + displacement * cos(displaced_rddf_pose.theta - M_PI / 2.0);
//				displaced_rddf_pose.y = displaced_rddf_pose.y + displacement * sin(displaced_rddf_pose.theta - M_PI / 2.0);
//				if (!trajectory_pose_hit_obstacle(displaced_rddf_pose, circle_radius, current_map, &robot_config))
//				{
//					rddf->poses[rddf_pose_index] = displaced_rddf_pose;
//					rddf_pose_hit_obstacle = 0;
//					break;
//				}
//			}
//		}
//	}
//
//	return (rddf_pose_hit_obstacle);
}


void
datmo_clear_detected()
{
	if (behavior_selector_use_symotha)
		udatmo_clear_detected();
}


void
datmo_shift_history()
{
	if (behavior_selector_use_symotha)
		udatmo_shift_history();
}


void
datmo_init(const carmen_robot_ackerman_config_t config)
{
	if (behavior_selector_use_symotha)
		udatmo_init(config);
}

double mo_v = 0.0;

double
datmo_speed_front()
{
	if (behavior_selector_use_symotha)
		return (udatmo_speed_front());
	else
		return (mo_v);
}


double mo_dist = 1000.0;

double
datmo_get_moving_obstacle_distance(carmen_ackerman_traj_point_t robot_pose, carmen_robot_ackerman_config_t *robot_config)
{
	if (behavior_selector_use_symotha)
		return (udatmo_get_moving_obstacle_distance(robot_pose, robot_config));
	else
		return (mo_dist);
}


int
datmo_detect_obstacle_index(carmen_obstacle_distance_mapper_map_message *current_map,
							carmen_rddf_road_profile_message *rddf,
							int goal_index,
							int rddf_pose_index,
							carmen_ackerman_traj_point_t robot_pose,
							carmen_moving_objects_point_clouds_message *current_moving_objects,
							double timestamp)
{
	if (behavior_selector_use_symotha)
	{
		int moving_object_in_front_index = udatmo_detect_obstacle_index(current_map, rddf, goal_index, rddf_pose_index, robot_pose, timestamp);
		return (moving_object_in_front_index);
	}
	else
	{
		if ((goal_index == 0) && current_moving_objects) // soh quando tem objetos moveis e estamos buscando o primeiro goal da goal_lis
		{
			for (int i = 0; i < current_moving_objects->num_point_clouds; i++)
			{
				t_point_cloud_struct mo = current_moving_objects->point_clouds[i];
				if (mo.in_front)
				{
					carmen_ackerman_traj_point_t cp = rddf->poses[rddf_pose_index];
					carmen_point_t car_pose = {cp.x, cp.y, cp.theta};
					carmen_point_t moving_object_pose = {mo.object_pose.x, mo.object_pose.y, mo.orientation};
					if (carmen_obstacle_avoider_car_collides_with_moving_object(car_pose, moving_object_pose, &mo, 0.0, 0.0))
					{
						mo_v = mo.linear_velocity;
						mo_dist = DIST2D(robot_pose, mo.object_pose);

						return (rddf_pose_index);
					}
				}
			}
		}
	}

	mo_v = 0.0;
	mo_dist = 1000.0;

	return (-1);
}


int
get_parameters_for_filling_in_goal_list(int &moving_object_in_front_index, int &last_obstacle_index, int &last_obstacle_free_waypoint_index,
		double &distance_from_car_to_rddf_point, double &distance_to_last_obstacle, double &distance_to_annotation,
		double &distance_to_last_obstacle_free_waypoint,
		carmen_rddf_road_profile_message *rddf, int rddf_pose_index, int goal_index,
		carmen_ackerman_traj_point_t current_goal, int current_goal_rddf_index,
		carmen_moving_objects_point_clouds_message *current_moving_objects,
		double timestamp, int &first_pose_change_direction_index)
{
	int rddf_pose_hit_obstacle = try_avoiding_obstacle(rddf_pose_index, robot_config.obstacle_avoider_obstacles_safe_distance, rddf);

	moving_object_in_front_index = datmo_detect_obstacle_index(current_map, rddf, goal_index, rddf_pose_index, robot_pose,
			current_moving_objects, timestamp);

	if (rddf_pose_hit_obstacle || (moving_object_in_front_index != -1))
		last_obstacle_index = rddf_pose_index;
	else if (last_obstacle_index == -1)
		last_obstacle_free_waypoint_index = rddf_pose_index;

	if (last_obstacle_index != -1)
		distance_to_last_obstacle = carmen_distance_ackerman_traj(&rddf->poses[last_obstacle_index], &rddf->poses[rddf_pose_index]);

	distance_from_car_to_rddf_point = 0.0;
	for (int i = current_goal_rddf_index; i < rddf_pose_index - 1; i++)
		distance_from_car_to_rddf_point += DIST2D(rddf->poses[i], rddf->poses[i + 1]);

	distance_to_annotation = carmen_distance_ackerman_traj(&current_goal, &rddf->poses[rddf_pose_index]);
	distance_to_last_obstacle_free_waypoint = carmen_distance_ackerman_traj(&current_goal, &rddf->poses[last_obstacle_free_waypoint_index]);

	if (behavior_selector_reverse_driving &&
		(rddf_pose_index < (rddf->number_of_poses - 1)) &&
		(carmen_sign(rddf->poses[rddf_pose_index].v) != carmen_sign(rddf->poses[rddf_pose_index + 1].v)))
		first_pose_change_direction_index = rddf_pose_index;
	else
		first_pose_change_direction_index = -1;

	return (rddf_pose_hit_obstacle);
}


bool
recent_moving_object_near_this_rddf_pose(carmen_ackerman_traj_point_t pose, carmen_ackerman_traj_point_t moving_obstacle_pose,
		double last_moving_obstacle_detection_timestamp, double timestamp)
{
//	printf("dist rddf-mo %lf\n", DIST2D(pose, moving_obstacle_pose));
	if (((fabs(timestamp - last_moving_obstacle_detection_timestamp)) < 2.0) &&
			(DIST2D(pose, moving_obstacle_pose) < 15.0))
		return (true);
	else
		return (false);
}


static void
clear_cells_below_robot(carmen_ackerman_traj_point_t pose)
{
	double delta_vertical_x, delta_vertical_y, delta_horizontal_x, delta_horizontal_y;
	carmen_point_t vertical_pose, horizontal_pose[2];
	int vertical_size, horizontal_size;

	if (current_map == NULL)
		return;

	vertical_size = ceil((robot_config.length) / current_map->config.resolution);
	horizontal_size = ceil(robot_config.model_predictive_planner_obstacles_safe_distance / current_map->config.resolution);

	delta_vertical_x = cos(pose.theta);
	delta_vertical_y = sin(pose.theta);

	delta_horizontal_x = cos(M_PI/2 - pose.theta);
	delta_horizontal_y = sin(M_PI/2 - pose.theta);

	vertical_pose.theta = pose.theta;
	vertical_pose.x = (pose.x - current_map->config.x_origin) / current_map->config.resolution;
	vertical_pose.y = (pose.y - current_map->config.y_origin) / current_map->config.resolution;

	vertical_pose.x -= robot_config.distance_between_rear_car_and_rear_wheels / current_map->config.resolution * cos(vertical_pose.theta);
	vertical_pose.y -= robot_config.distance_between_rear_car_and_rear_wheels / current_map->config.resolution * sin(vertical_pose.theta);

	for (int v = 0; v <= vertical_size; v++)
	{
		horizontal_pose[0] = vertical_pose;
		horizontal_pose[1] = vertical_pose;

		for (int h = 0; h <= horizontal_size; h++)
		{
			for (int i = 0; i < 2; i++)
			{
				if (horizontal_pose[i].x >= 0 && horizontal_pose[i].x < current_map->config.x_size &&
						horizontal_pose[i].y >= 0 && horizontal_pose[i].y < current_map->config.y_size)
				{
					current_map->complete_x_offset[(int) horizontal_pose[i].x * current_map->config.y_size + (int) horizontal_pose[i].y] = DISTANCE_MAP_HUGE_DISTANCE;
					current_map->complete_y_offset[(int) horizontal_pose[i].x * current_map->config.y_size + (int) horizontal_pose[i].y] = DISTANCE_MAP_HUGE_DISTANCE;
				}
			}

			horizontal_pose[0].x = horizontal_pose[0].x - delta_horizontal_x;
			horizontal_pose[0].y = horizontal_pose[0].y + delta_horizontal_y;

			horizontal_pose[1].x = horizontal_pose[1].x + delta_horizontal_x;
			horizontal_pose[1].y = horizontal_pose[1].y - delta_horizontal_y;
		}

		vertical_pose.x = vertical_pose.x + delta_vertical_x;
		vertical_pose.y = vertical_pose.y + delta_vertical_y;
	}
}


void
clear_lane_ahead_in_distance_map(int current_goal_rddf_index, int ideal_rddf_pose_index, carmen_rddf_road_profile_message *rddf)
{
	int begin = current_goal_rddf_index;
	while ((DIST2D(rddf->poses[begin], rddf->poses[current_goal_rddf_index]) < robot_config.model_predictive_planner_obstacles_safe_distance) &&
		   (begin > 0))
		begin--;

	int end = ideal_rddf_pose_index;
	while ((DIST2D(rddf->poses[end], rddf->poses[ideal_rddf_pose_index]) < robot_config.model_predictive_planner_obstacles_safe_distance) &&
		   (end < rddf->number_of_poses))
		end++;

	for (int i = begin; i < end; i++)
		clear_cells_below_robot(rddf->poses[i]);
}


int
behavior_selector_set_task(carmen_behavior_selector_task_t task)
{
	if (task == current_task)
		return (0);

	current_task = task;

	return (1);
}


void
behavior_selector_get_full_state(carmen_behavior_selector_task_t *current_task_out, carmen_behavior_selector_algorithm_t *following_lane_planner_out,
		carmen_behavior_selector_algorithm_t *parking_planner_out, carmen_behavior_selector_goal_source_t *current_goal_source_out)
{
	*current_task_out = current_task;
	*following_lane_planner_out = following_lane_planner;
	*parking_planner_out = parking_planner;
	*current_goal_source_out = current_goal_source;
}


int
behavior_selector_get_task()
{
	return (current_task);
}


int
behavior_selector_set_goal_source(carmen_behavior_selector_goal_source_t goal_source)
{
	if (current_goal_source == goal_source)
		return (0);

	current_goal_source = goal_source;

	goal_list_size = 0;

	return (1);
}


void
behavior_selector_add_goal(carmen_point_t goal)
{
	goal_list[goal_list_size].x = goal.x;
	goal_list[goal_list_size].y = goal.y;
	goal_list[goal_list_size].theta = goal.theta;
	goal_list[goal_list_size].v = 0.0;

	goal_list_size++;
}


void
behavior_selector_clear_goal_list()
{
	goal_list_size = 0;
}


void
behavior_selector_remove_goal()
{
	goal_list_size--;
	if (goal_list_size < 0)
		goal_list_size = 0;
}


carmen_ackerman_traj_point_t *
behavior_selector_get_last_goal_list(int &last_goal_list_size)
{
	last_goal_list_size = goal_list_size;

	return (goal_list);
}


carmen_ackerman_traj_point_t *
behavior_selector_get_last_goals_and_types(int *&goals_types, int &last_goal_list_size)
{
	last_goal_list_size = goal_list_size;
	goals_types = goal_type;

	return (goal_list);
}


int
behavior_selector_get_last_goal_type()
{
	return (goal_type[0]);
}


void
behavior_selector_set_algorithm(carmen_behavior_selector_algorithm_t algorithm, carmen_behavior_selector_task_t task)
{
	switch(task)
	{
	case BEHAVIOR_SELECTOR_FOLLOW_ROUTE:
		following_lane_planner = algorithm;
		break;

	case BEHAVIOR_SELECTOR_PARK:
		parking_planner = algorithm;
		break;
	default:
		following_lane_planner = algorithm;
		break;
	}
}


void
behavior_selector_update_robot_pose(carmen_ackerman_traj_point_t pose)
{
	if (carmen_distance_ackerman_traj(&robot_pose, &pose) > 2.5 && current_goal_source != CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
		goal_list_size = 0; //provavelmente o robo foi reposicionado

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


void
set_max_v(double v)
{
	robot_config.max_v = v;
}


double
get_max_v_reverse()
{//TODO usar essa variavel dentro do robot_config
	return (robot_max_velocity_reverse);
}


void
set_max_v_reverse(double v)
{
	//TODO
	robot_max_velocity_reverse = v;
}


carmen_robot_ackerman_config_t *
get_robot_config()
{
	return (&robot_config);
}


void
behavior_selector_update_map(carmen_obstacle_distance_mapper_map_message *map)
{
	current_map = map;
}


void
change_distance_between_waypoints_and_goals(double dist_between_waypoints, double change_goal_dist)
{
	distance_between_waypoints = dist_between_waypoints;
	change_goal_distance = change_goal_dist;
}


double
distance_between_waypoints_and_goals()
{
	return (distance_between_waypoints);
}


double
compute_final_velocity_using_torricelli(double current_velocity, double acceleration, double poses_displacement)
{
	double vel = (current_velocity * current_velocity) + (2 * acceleration * poses_displacement);

	if (vel > 0.0)
		return (sqrt(vel));
	
	return (0.0);
}


int
compute_stop_pose_rddf_index(carmen_ackerman_traj_point_t robot_state, carmen_rddf_road_profile_message *rddf_vector, int rddf_size, 
	double deceleration, double robot_velocity_zero_bias)
{
	rddf_size = rddf_size - 1;        // To deal with i + 1
	double final_v = robot_state.v;

	for (int i = 0; i < rddf_size; i++)
	{
		final_v = compute_final_velocity_using_torricelli(final_v, deceleration, DIST2D(rddf_vector->poses[i], rddf_vector->poses[i + 1]));

		// printf("FV %lf %lf %lf\n", robot_state.v, final_v, DIST2D(rddf_vector->poses[i], rddf_vector->poses[i + 1]));

		if (final_v < robot_velocity_zero_bias)
		{
			return (i);
		}
	}
	return (-1);
}


void
behavior_selector_initialize(carmen_robot_ackerman_config_t config, double dist_between_waypoints, double change_goal_dist,
		carmen_behavior_selector_algorithm_t f_planner, carmen_behavior_selector_algorithm_t p_planner, double max_velocity_reverse)
{
	datmo_init(config);

	robot_config = config;
	robot_max_velocity_reverse = max_velocity_reverse;
	distance_between_waypoints = dist_between_waypoints;
	change_goal_distance = change_goal_dist;
	parking_planner = p_planner;
	following_lane_planner = f_planner;
	//memset(moving_object, 0, sizeof(MOVING_OBJECT) * MOVING_OBJECT_HISTORY_SIZE);

	memset(&virtual_laser_message, 0, sizeof(carmen_mapper_virtual_laser_message));
	virtual_laser_message.positions = (carmen_position_t *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(carmen_position_t));
	virtual_laser_message.colors = (char *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(char));
	virtual_laser_message.host = carmen_get_host();

	//SampleFilter_init(&filter);
	SampleFilter_init(&filter2);
}


carmen_ackerman_traj_point_t *
set_goal_list(int &current_goal_list_size, carmen_ackerman_traj_point_t *&first_goal, int &first_goal_type,
		carmen_rddf_road_profile_message *rddf, path_collision_info_t path_collision_info,
		carmen_moving_objects_point_clouds_message *current_moving_objects,
		carmen_behavior_selector_state_message behavior_selector_state_message, double timestamp)
{
	double distance_to_last_obstacle = 10000.0;
	int last_obstacle_index = -1;

	current_goal_list_size = 0;
	datmo_clear_detected();
	datmo_shift_history();

	int goal_index = 0;
	carmen_ackerman_traj_point_t current_goal = robot_pose;
	int current_goal_rddf_index = 0;
//	virtual_laser_message.num_positions = 0;
//	printf("v %lf\n", datmo_speed_front());
	int last_obstacle_free_waypoint_index = 0;

	double distance_car_pose_car_front = robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels;

	int first_pose_change_direction_index = -1;

	static carmen_ackerman_traj_point_t previous_first_goal;


#ifdef PRINT_UDATMO_LOG

	carmen_ackerman_traj_point_t front_obst = udatmo_get_moving_obstacle_position();
	double front_obst_velocity = udatmo_speed_front();

	carmen_ackerman_traj_point_t left_obst = udatmo_get_moving_obstacle_position_left();
	double left_obst_velocity = udatmo_speed_left();

	carmen_ackerman_traj_point_t right_obst = udatmo_get_moving_obstacle_position_right();
	double right_obst_velocity = udatmo_speed_right();

	printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", timestamp, robot_pose.x, robot_pose.y, robot_pose.v, robot_pose.theta, robot_pose.phi,
			front_obst.x, front_obst.y, front_obst_velocity,
			left_obst.x, left_obst.y, left_obst_velocity,
			right_obst.x, right_obst.y, right_obst_velocity);
#endif
//	for (int rddf_pose_index = 0; rddf_pose_index < rddf->number_of_poses; rddf_pose_index++)
//		printf("%lf %lf %lf %lf\n",
//			rddf->poses[rddf_pose_index].v,
//			rddf->poses[rddf_pose_index].x,
//			rddf->poses[rddf_pose_index].y,
//			rddf->poses[rddf_pose_index].theta);
//	printf("\n");

	for (int rddf_pose_index = 0; rddf_pose_index < rddf->number_of_poses && goal_index < GOAL_LIST_SIZE; rddf_pose_index++)
	{

		double distance_from_car_to_rddf_point, distance_to_annotation, distance_to_last_obstacle_free_waypoint;
		int rddf_pose_hit_obstacle, moving_object_in_front_index;


		rddf_pose_hit_obstacle = get_parameters_for_filling_in_goal_list(moving_object_in_front_index, last_obstacle_index,
				last_obstacle_free_waypoint_index, distance_from_car_to_rddf_point, distance_to_last_obstacle, distance_to_annotation,
				distance_to_last_obstacle_free_waypoint, rddf, rddf_pose_index, goal_index, current_goal, current_goal_rddf_index,
				current_moving_objects, timestamp, first_pose_change_direction_index);

		static double moving_obstacle_trasition = 0.0;
		if (rddf_pose_hit_obstacle)
		{
			goal_type[goal_index] = OBSTACLE_GOAL;
			double distance_to_free_waypoint = DIST2D(rddf->poses[0], rddf->poses[last_obstacle_free_waypoint_index]);
			double reduction_factor = (fabs(robot_pose.v) > 1.0)? 1.0 / fabs(robot_pose.v): 1.0;
			if (distance_to_free_waypoint >= (distance_car_pose_car_front / 2.0))
			{
				double displacement = ((distance_car_pose_car_front / 2.0) * reduction_factor);

				if(rddf->poses[rddf_pose_index].v >= 0.0)
					displacement = -displacement;

				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, last_obstacle_free_waypoint_index, rddf,
						displacement);
			}
			else
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, 0, rddf);
			moving_obstacle_trasition = 0.0;
			break;
		}
		else if (moving_object_in_front_index != -1) // -> Adiciona um waypoint na ultima posicao livre se a posicao atual colide com um objeto movel.
		{
			double d = 0;
			int ideal_rddf_pose_index;
			for (ideal_rddf_pose_index = current_goal_rddf_index; ideal_rddf_pose_index < rddf->number_of_poses - 1; ideal_rddf_pose_index++)
			{
				d += DIST2D(rddf->poses[ideal_rddf_pose_index], rddf->poses[ideal_rddf_pose_index + 1]);
				if (d > (distance_between_waypoints - (distance_car_pose_car_front / 2.0)))
					break;
			}

			double reduction_factor = (robot_pose.v > 1.0)? 1.0 / robot_pose.v: 1.0;
			if ((moving_obstacle_trasition != 0.0) || (robot_pose.v > datmo_speed_front()) || (datmo_speed_front() < 2.0))
			{
				goal_type[goal_index] = MOVING_OBSTACLE_GOAL1;
				moving_obstacle_trasition += 1.0 / (3.0 * 20.0);
				if ((moving_obstacle_trasition > 1.0) || (datmo_speed_front() < 2.0))
					moving_obstacle_trasition = 1.0;

				int adequate_rddf_index = ideal_rddf_pose_index;
				if (ideal_rddf_pose_index > last_obstacle_free_waypoint_index)
					adequate_rddf_index -= round((ideal_rddf_pose_index - last_obstacle_free_waypoint_index) * moving_obstacle_trasition);

				double distance_to_free_waypoint = DIST2D(rddf->poses[0], rddf->poses[adequate_rddf_index]);
				if (moving_obstacle_trasition != 1.0)
//					clear_lane_ahead_in_distance_map(rddf_pose_index, ideal_rddf_pose_index, rddf);
					if (behavior_selector_use_symotha)
						clear_lane_ahead_in_distance_map(current_goal_rddf_index, ideal_rddf_pose_index, rddf);

				if (distance_to_free_waypoint >= (distance_car_pose_car_front / 2.0))
					add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, adequate_rddf_index, rddf,
							-(distance_car_pose_car_front / 2.0) * reduction_factor);
				else
					add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, 0, rddf);
			}
			else
			{
				goal_type[goal_index] = MOVING_OBSTACLE_GOAL2;
//				clear_lane_ahead_in_distance_map(rddf_pose_index, ideal_rddf_pose_index, rddf);
				if (behavior_selector_use_symotha)
					clear_lane_ahead_in_distance_map(current_goal_rddf_index, ideal_rddf_pose_index, rddf);
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, ideal_rddf_pose_index, rddf,
						-(distance_car_pose_car_front / 2.0) * reduction_factor);
			}
			break;
		}
		else if (must_yield(path_collision_info, timestamp) &&
				 (rddf_pose_index == path_collision_info.possible_collision_mo_pose_index))
		{
			goal_type[goal_index] = MOVING_OBSTACLE_GOAL3;
			add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, last_obstacle_free_waypoint_index, rddf);
			moving_obstacle_trasition = 0.0;
		}
		else if ((distance_from_car_to_rddf_point > (map_width / 3.0 - distance_car_pose_car_front)) ||
				 ((rddf_pose_hit_obstacle == 2) && (rddf_pose_index > 10))) // Goal esta fora do mapa
		{
			goal_type[goal_index] = FREE_RUN_GOAL2;
			add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, last_obstacle_free_waypoint_index, rddf);
			moving_obstacle_trasition = 0.0;
			break;
		}
//		else if (rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_DYNAMIC)
//		{
//			goal_type[goal_index] = DYNAMIC_ANNOTATION_GOAL;
//			add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, rddf_pose_index, rddf);
//			break;
//		}
		else if ((((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BARRIER)) && // || // -> Adiciona um waypoint na posicao atual se ela contem uma das anotacoes especificadas
//				   (rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BUMP)) &&
				  (distance_to_annotation > distance_to_remove_annotation_goal) && // e se ela esta a uma distancia apropriada da anotacao
				  !rddf_pose_hit_obstacle)) // e se ela nao colide com um obstaculo.
		{
			goal_type[goal_index] = ANNOTATION_GOAL1;
			double distance_to_waypoint = DIST2D(rddf->poses[0], rddf->poses[rddf_pose_index]);
			if (distance_to_waypoint >= 0.0)
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, rddf_pose_index, rddf, 0.0);
			else
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, 0, rddf);
			moving_obstacle_trasition = 0.0;
		}
		else if (((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_STOP) &&  // -> Adiciona um waypoint na posicao atual se ela contem uma das anotacoes especificadas
				   ((behavior_selector_state_message.low_level_state == Stopping_At_Stop_Sign) ||
					(behavior_selector_state_message.low_level_state == Stopped_At_Stop_Sign_S0) ||
					(behavior_selector_state_message.low_level_state == Stopped_At_Stop_Sign_S1))) &&
				  !rddf_pose_hit_obstacle) // e se ela nao colide com um obstaculo.
		{
			goal_type[goal_index] = ANNOTATION_GOAL_STOP;
			double distance_to_waypoint = DIST2D(rddf->poses[0], rddf->poses[rddf_pose_index]);
			if (distance_to_waypoint >= 0.0)
			{
				double displacement = 0.5;
				if (rddf->poses[rddf_pose_index].v >= 0.0)
					displacement = -displacement;
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, rddf_pose_index, rddf, displacement);
			}
			else
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, 0, rddf);
			moving_obstacle_trasition = 0.0;
		}
		else if ((((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_YIELD) &&
				   (must_yield(path_collision_info, timestamp) ||
					(behavior_selector_state_message.low_level_state == Stopping_At_Yield) ||
					(behavior_selector_state_message.low_level_state == Stopped_At_Yield_S0) ||
					(behavior_selector_state_message.low_level_state == Stopped_At_Yield_S1))) ||
				  ((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP) &&
				   (red_traffic_light_ahead(robot_pose, timestamp) ||
					(behavior_selector_state_message.low_level_state == Stopping_At_Red_Traffic_Light) ||
					(behavior_selector_state_message.low_level_state == Stopped_At_Red_Traffic_Light_S0) ||
					(behavior_selector_state_message.low_level_state == Stopped_At_Red_Traffic_Light_S1))) ||
				  ((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP) &&
				   (busy_pedestrian_track_ahead(robot_pose, timestamp) ||
					(behavior_selector_state_message.low_level_state == Stopping_At_Busy_Pedestrian_Track) ||
					(behavior_selector_state_message.low_level_state == Stopped_At_Busy_Pedestrian_Track_S0) ||
					(behavior_selector_state_message.low_level_state == Stopped_At_Busy_Pedestrian_Track_S1)))) &&
				  !rddf_pose_hit_obstacle) // e se ela nao colide com um obstaculo.
		{
			goal_type[goal_index] = ANNOTATION_GOAL2;
			double distance_to_waypoint = DIST2D(rddf->poses[0], rddf->poses[rddf_pose_index]);
			if (distance_to_waypoint >= 0.0)
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, rddf_pose_index, rddf, 0.0);
			else
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, 0, rddf);
			moving_obstacle_trasition = 0.0;
		}
		else if (((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BARRIER) || // -> Adiciona um waypoint na ultima posicao livre se a posicao atual contem uma das anotacoes especificadas
//				  (rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BUMP) ||
				  ((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_STOP) && !wait_start_moving) ||
				  ((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_YIELD) && !wait_start_moving) ||
				  ((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP) && !wait_start_moving) ||
				  ((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP) && !wait_start_moving)) &&
				 (distance_to_last_obstacle_free_waypoint > 1.5) && // e se ela esta a mais de 1.5 metros da ultima posicao livre de obstaculo
				 rddf_pose_hit_obstacle) // e se ela colidiu com obstaculo.
		{	// Ou seja, se a anotacao estiver em cima de um obstaculo, adiciona um waypoint na posicao anterior mais proxima da anotacao que estiver livre.
			goal_type[goal_index] = ANNOTATION_GOAL3;
			add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, last_obstacle_free_waypoint_index, rddf);
			moving_obstacle_trasition = 0.0;
		}
		else if ((((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK) &&  // -> Adiciona um waypoint na ultima posicao livre se a posicao atual contem uma das anotacoes especificadas
				   !wait_start_moving && busy_pedestrian_track_ahead(robot_pose, timestamp))) &&
				   !rddf_pose_hit_obstacle)
		{
			goal_type[goal_index] = ANNOTATION_GOAL2;
			double distance_to_waypoint = DIST2D(rddf->poses[0], rddf->poses[rddf_pose_index]); // Distancia da extremidade dianteira do robo para a anotacao
			// printf ("ENtrou  ");
			if (distance_to_waypoint > (MIN_DISTANCE_TO_CONSIDER_CROSSWALK + robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels)) // Se passou deste ponto, o robo ja está muito encima da faixa e nao adianta mais parar
			{
				int	goal_in_pedestrian_track_rddf_index = compute_stop_pose_rddf_index(robot_pose, rddf, rddf->number_of_poses, -1.0, 0.2);  ///////////// TODO ler a desaceleracao e o zero bias do carmen ini
				// printf ("RDDF_I %d %d %lf %lf\n", rddf_pose_index, goal_in_pedestrian_track_rddf_index, (1.5 + robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels), distance_to_waypoint);
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, goal_in_pedestrian_track_rddf_index, rddf);
			}
			moving_obstacle_trasition = 0.0;
		}
		else if ((((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK) && !wait_start_moving)) &&  // -> Adiciona um waypoint na ultima posicao livre se a posicao atual contem uma das anotacoes especificadas
				 (distance_to_last_obstacle_free_waypoint > 1.5) && // e se ela esta a mais de 1.5 metros da ultima posicao livre de obstaculo
				 rddf_pose_hit_obstacle) // e se ela colidiu com obstaculo.
		{	// Ou seja, se a anotacao estiver em cima de um obstaculo, adiciona um waypoint na posicao anterior mais proxima da anotacao que estiver livre.

			goal_type[goal_index] = ANNOTATION_GOAL3;
			double displacement = ((rddf->poses[rddf_pose_index].v >= 0.0) ? -2.0 : 2.0);
			add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, last_obstacle_free_waypoint_index, rddf, displacement);
			moving_obstacle_trasition = 0.0;
		}
		//parking se pose anterior foi em uma direcao, aguarda o carro terminar todo o path dessa direcao antes de mudar de direcao (reh/drive)
		else if (behavior_selector_reverse_driving &&
				 ((first_pose_change_direction_index != -1)))//	&&
//				  (DIST2D(rddf->poses[rddf_pose_index], robot_pose) > 1.0)))
		{
			static double keep_goal_time = 0.0;
			if (first_pose_change_direction_index > 2)
			{
				if (goal_index == 0)
					keep_goal_time = 0.0;

				goal_type[goal_index] = SWITCH_VELOCITY_SIGNAL_GOAL;
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, rddf_pose_index, rddf);
			}
			else
			{
				if ((fabs(robot_pose.v) < 0.05) && (keep_goal_time == 0.0))
					keep_goal_time = carmen_get_time();

				if (keep_goal_time == 0.0)
				{
					goal_type[goal_index] = SWITCH_VELOCITY_SIGNAL_GOAL;
					add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, rddf_pose_index, rddf);
				}
				else if ((carmen_get_time() - keep_goal_time) < 3.5)	// Espera parado um pouco
				{
					goal_type[goal_index] = SWITCH_VELOCITY_SIGNAL_GOAL;
					add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, rddf_pose_index, rddf);
				}
			}
		}
		else if (((distance_from_car_to_rddf_point >= distance_between_waypoints) ||  // -> Adiciona um waypoint na posicao atual se ela esta numa distancia apropriada
				  ((distance_from_car_to_rddf_point >= distance_between_waypoints / 2.0) && (rddf->poses[rddf_pose_index].v < 0.0))) && // -> Trocando a constante que divide distance_between_waypoints pode-se alterar a distância entre waypoints em caso de reh
				  (distance_to_last_obstacle >= 15.0) && // e se ela esta pelo menos 15.0 metros aa frente de um obstaculo
				  !rddf_pose_hit_obstacle) // e se ela nao colide com um obstaculo.
		{
			goal_type[goal_index] = FREE_RUN_GOAL1;
			add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, rddf_pose_index, rddf);
			moving_obstacle_trasition = 0.0;
		}

		else if ((rddf_pose_index == (rddf->number_of_poses - 1)) &&
				 !rddf_pose_hit_obstacle) //-> Se o ultimo ponto do path eh o final goal, adiciona o goal la
		{
			goal_type[goal_index] = FINAL_GOAL;
			add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, rddf_pose_index, rddf);
		}
	}

//	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, timestamp);

	goal_list_size = current_goal_list_size = goal_index;

	// printf("Goal List: ");
	// for (int i = 0; i < goal_list_size; i++)
	// 	printf("%lf  ", goal_list[i].v);

	// printf("\n");

	first_goal = &(goal_list[0]);
	first_goal_type = goal_type[0];
	previous_first_goal = goal_list[0];

	return (goal_list);
}

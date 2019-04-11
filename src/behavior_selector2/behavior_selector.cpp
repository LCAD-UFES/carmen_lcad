/*
 * behavior_selector.c
 *
 *  Created on: 28/09/2012
 *      Author: romulo
 */

#include <carmen/collision_detection.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/udatmo.h>
#include <carmen/global_graphics.h>
#include "behavior_selector.h"
#include "behavior_selector_messages.h"

#define GOAL_LIST_SIZE 1000
#define MAX_ANNOTATIONS 50

//#define PRINT_UDATMO_LOG
#define USE_DATMO_GOAL

static carmen_robot_ackerman_config_t robot_config;
static double distance_between_waypoints = 5;
static carmen_ackerman_traj_point_t robot_pose;
static int robot_initialized = 0;
static carmen_ackerman_traj_point_t goal_list[GOAL_LIST_SIZE];
static int goal_type[GOAL_LIST_SIZE];
static int annotations[GOAL_LIST_SIZE];
static int goal_list_size = 0;
static carmen_obstacle_distance_mapper_map_message *current_map = NULL;
static carmen_behavior_selector_state_t current_state = BEHAVIOR_SELECTOR_FOLLOWING_LANE;
static carmen_behavior_selector_goal_source_t current_goal_source = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
static double change_goal_distance = 8.0; // @@@ Alberto: acho que nao usa... deletar?
static carmen_behavior_selector_algorithm_t following_lane_planner;
static carmen_behavior_selector_algorithm_t parking_planner;
static double distance_to_remove_annotation_goal = 1.5;

int position_of_next_annotation = 0;


// filipe:: TODO: colocar no carmen.ini
double dist_to_reduce_speed = 15.0;
double speed_around_annotation = 1.0;

//MOVING_OBJECT moving_object[MOVING_OBJECT_HISTORY_SIZE];
//int moving_object_in_front_detected = 0;

#define MAX_VIRTUAL_LASER_SAMPLES 10000
carmen_mapper_virtual_laser_message virtual_laser_message;

//SampleFilter filter;
SampleFilter filter2;

extern bool wait_start_moving;

extern double map_width;


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


void
change_state(int rddf_annotation)
{
	if (current_goal_source == CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
		return;

	switch(rddf_annotation)
	{
//	case RDDF_ANNOTATION_TYPE_NONE:
//		current_state = BEHAVIOR_SELECTOR_FOLLOWING_LANE;
//		break;

	case RDDF_ANNOTATION_TYPE_END_POINT_AREA:
		current_state = BEHAVIOR_SELECTOR_PARKING;
		break;

	case RDDF_ANNOTATION_TYPE_HUMAN_INTERVENTION:
		current_state = BEHAVIOR_SELECTOR_HUMAN_INTERVENTION;
		carmen_navigator_ackerman_stop();
		break;
	}
}


void
add_goal_to_goal_list(int &goal_index, carmen_ackerman_traj_point_t &current_goal, int &current_goal_rddf_index, int rddf_pose_index,
		carmen_rddf_road_profile_message *rddf)
{
	goal_list[goal_index] = rddf->poses[rddf_pose_index];
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

	goal_list[goal_index] = new_car_traj_point;
	annotations[goal_index] = rddf->annotations[rddf_pose_index];
	current_goal = new_car_traj_point;
	current_goal_rddf_index = rddf_pose_index;
	goal_index++;
}


int
try_avoiding_obstacle(int rddf_pose_index, double circle_radius, carmen_rddf_road_profile_message* rddf)
{
	//int rddf_pose_hit_obstacle = trajectory_pose_hit_obstacle_new(rddf->poses[rddf_pose_index], current_map, &robot_config);
	int rddf_pose_hit_obstacle = trajectory_pose_hit_obstacle(rddf->poses[rddf_pose_index], circle_radius, current_map, &robot_config);

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


int
get_parameters_for_filling_in_goal_list(int &moving_object_in_front_index, int &last_obstacle_index, int &last_obstacle_free_waypoint_index,
		double &distance_from_car_to_rddf_point, double &distance_to_last_obstacle, double &distance_to_annotation,
		double &distance_to_last_obstacle_free_waypoint,
		carmen_rddf_road_profile_message *rddf, int rddf_pose_index, int goal_index,
		carmen_ackerman_traj_point_t current_goal, int current_goal_rddf_index,
		double timestamp)
{
	int rddf_pose_hit_obstacle = try_avoiding_obstacle(rddf_pose_index, robot_config.obstacle_avoider_obstacles_safe_distance, rddf);

	moving_object_in_front_index = udatmo_detect_obstacle_index(current_map, rddf, goal_index, rddf_pose_index, robot_pose, timestamp);

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

	return (rddf_pose_hit_obstacle);
}


bool
recent_moving_object_near_this_rddf_pose(carmen_ackerman_traj_point_t pose, carmen_ackerman_traj_point_t moving_obstacle_pose,
		double last_moving_obstacle_detection_timestamp, double timestamp)
{
	printf("dist rddf-mo %lf\n", DIST2D(pose, moving_obstacle_pose));
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
behaviour_selector_fill_goal_list(carmen_rddf_road_profile_message *rddf, double timestamp)
{
	double distance_to_last_obstacle = 10000.0;
	int last_obstacle_index = -1;

	goal_list_size = 0;
	udatmo_clear_detected();

	if (rddf == NULL)
		return (0);

	udatmo_shift_history();
	int goal_index = 0;
	carmen_ackerman_traj_point_t current_goal = robot_pose;
	int current_goal_rddf_index = 0;
//	virtual_laser_message.num_positions = 0;
//	printf("v %lf\n", udatmo_speed_front());
	int last_obstacle_free_waypoint_index = 0;
	double distance_car_pose_car_front = robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels;

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

	for (int rddf_pose_index = 0; rddf_pose_index < rddf->number_of_poses && goal_index < GOAL_LIST_SIZE; rddf_pose_index++)
	{
		double distance_from_car_to_rddf_point, distance_to_annotation, distance_to_last_obstacle_free_waypoint;
		int rddf_pose_hit_obstacle, moving_object_in_front_index;


		rddf_pose_hit_obstacle = get_parameters_for_filling_in_goal_list(moving_object_in_front_index, last_obstacle_index,
				last_obstacle_free_waypoint_index, distance_from_car_to_rddf_point, distance_to_last_obstacle, distance_to_annotation,
				distance_to_last_obstacle_free_waypoint,
				rddf, rddf_pose_index, goal_index, current_goal, current_goal_rddf_index, timestamp);
#ifndef USE_DATMO_GOAL
		rddf_pose_hit_obstacle = 0;
#endif

		static double moving_obstacle_trasition = 0.0;
#ifdef USE_DATMO_GOAL
		if (moving_object_in_front_index != -1) // -> Adiciona um waypoint na ultima posicao livre se a posicao atual colide com um objeto movel.
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
			if ((moving_obstacle_trasition != 0.0) || (robot_pose.v > udatmo_speed_front()) || (udatmo_speed_front() < 2.0))
			{
				goal_type[goal_index] = MOVING_OBSTACLE_GOAL1;
				moving_obstacle_trasition += 1.0 / (10.0 * 20.0);
				if ((moving_obstacle_trasition > 1.0) || (udatmo_speed_front() < 2.0))
					moving_obstacle_trasition = 1.0;

				int adequate_rddf_index = ideal_rddf_pose_index;
				if (ideal_rddf_pose_index > last_obstacle_free_waypoint_index)
					adequate_rddf_index -= round((ideal_rddf_pose_index - last_obstacle_free_waypoint_index) * moving_obstacle_trasition);

				double distance_to_free_waypoint = DIST2D(rddf->poses[0], rddf->poses[adequate_rddf_index]);
				if (moving_obstacle_trasition != 1.0)
//					clear_lane_ahead_in_distance_map(rddf_pose_index, ideal_rddf_pose_index, rddf);
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
				clear_lane_ahead_in_distance_map(current_goal_rddf_index, ideal_rddf_pose_index, rddf);
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, ideal_rddf_pose_index, rddf,
						-(distance_car_pose_car_front / 2.0) * reduction_factor);
			}
			break;
		}
#else
		if (0) {

		}
#endif

		else if ((distance_from_car_to_rddf_point > (map_width / 3.0 - distance_car_pose_car_front)) ||
				 ((rddf_pose_hit_obstacle == 2) && (rddf_pose_index > 10))) // Goal esta fora do mapa
		{
			goal_type[goal_index] = FREE_RUN_GOAL2;
			add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, last_obstacle_free_waypoint_index, rddf);
			moving_obstacle_trasition = 0.0;
			break;
		}
		else if (rddf_pose_hit_obstacle)
		{
			goal_type[goal_index] = OBSTACLE_GOAL;
			double distance_to_free_waypoint = DIST2D(rddf->poses[0], rddf->poses[last_obstacle_free_waypoint_index]);
			double reduction_factor = (robot_pose.v > 1.0)? 1.0 / robot_pose.v: 1.0;
			if (distance_to_free_waypoint >= (distance_car_pose_car_front / 2.0))
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, last_obstacle_free_waypoint_index, rddf,
						-(distance_car_pose_car_front / 2.0) * reduction_factor);
			else
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, 0, rddf);
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
		else if ((((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_STOP) &&  // -> Adiciona um waypoint na posicao atual se ela contem uma das anotacoes especificadas
				   !wait_start_moving && stop_sign_ahead(robot_pose)) ||
				  ((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP) &&
				   !wait_start_moving && red_traffic_light_ahead(robot_pose, timestamp)) ||
				  ((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK_STOP) &&
				   !wait_start_moving && busy_pedestrian_track_ahead(robot_pose, timestamp))) &&
				  !rddf_pose_hit_obstacle) // e se ela nao colide com um obstaculo.
		{
			goal_type[goal_index] = ANNOTATION_GOAL2;
			double distance_to_waypoint = DIST2D(rddf->poses[0], rddf->poses[rddf_pose_index]);
			if (distance_to_waypoint >= 0.0)
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, rddf_pose_index, rddf, 0.0);
			else
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, 0, rddf);
			moving_obstacle_trasition = 0.0;
			break;
		}
		else if (((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BARRIER) || // -> Adiciona um waypoint na ultima posicao livre se a posicao atual contem uma das anotacoes especificadas
//				  (rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BUMP) ||
				  ((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_STOP) && !wait_start_moving) ||
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
				  !rddf_pose_hit_obstacle) // e se ela nao colide com um obstaculo.
		{
			goal_type[goal_index] = ANNOTATION_GOAL2;
			double distance_to_waypoint = DIST2D(rddf->poses[0], rddf->poses[rddf_pose_index]);
			if (distance_to_waypoint >= 0.0)
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, rddf_pose_index, rddf, -2.0);
			else
				add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, 0, rddf);
			moving_obstacle_trasition = 0.0;
			break;
		}
		else if ((((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK) && !wait_start_moving)) &&  // -> Adiciona um waypoint na ultima posicao livre se a posicao atual contem uma das anotacoes especificadas
				 (distance_to_last_obstacle_free_waypoint > 1.5) && // e se ela esta a mais de 1.5 metros da ultima posicao livre de obstaculo
				 rddf_pose_hit_obstacle) // e se ela colidiu com obstaculo.
		{	// Ou seja, se a anotacao estiver em cima de um obstaculo, adiciona um waypoint na posicao anterior mais proxima da anotacao que estiver livre.
			goal_type[goal_index] = ANNOTATION_GOAL3;
			add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, last_obstacle_free_waypoint_index, rddf, -2.0);
			moving_obstacle_trasition = 0.0;
		}
		else if (((distance_from_car_to_rddf_point >= distance_between_waypoints) && // -> Adiciona um waypoint na posicao atual se ela esta numa distancia apropriada
				  (distance_to_last_obstacle >= 15.0) && // e se ela esta pelo menos 15.0 metros aa frente de um obstaculo
				  !rddf_pose_hit_obstacle)) // e se ela nao colide com um obstaculo.
		{
			goal_type[goal_index] = FREE_RUN_GOAL1;
			add_goal_to_goal_list(goal_index, current_goal, current_goal_rddf_index, rddf_pose_index, rddf);
			moving_obstacle_trasition = 0.0;
		}
	}

//	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, timestamp);

	goal_list_size = goal_index;

	return (1);
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

	carmen_ackerman_traj_point_t *goal_list_out = goal_list;

	return (goal_list_out);
}


int *
behavior_selector_get_goal_type()
{
	return (goal_type);
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


void
behavior_selector_initialize(carmen_robot_ackerman_config_t config, double dist_between_waypoints, double change_goal_dist,
		carmen_behavior_selector_algorithm_t f_planner, carmen_behavior_selector_algorithm_t p_planner)
{
	udatmo_init(config);

	robot_config = config;
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

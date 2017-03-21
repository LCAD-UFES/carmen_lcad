/*
 * behavior_selector.c
 *
 *  Created on: 28/09/2012
 *      Author: romulo
 */

#include <carmen/collision_detection.h>
#include <carmen/cpp_debug_log.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/udatmo_api.h>
#include <carmen/udatmo_interface.h>
#include <carmen/global_graphics.h>
#include "behavior_selector.h"
#include "behavior_selector_messages.h"

#define MAX_GOAL_LIST_SIZE 1000

static carmen_robot_ackerman_config_t robot_config;
static double distance_between_waypoints = 5;
static carmen_ackerman_traj_point_t robot_pose;
static int robot_initialized = 0;
static carmen_ackerman_traj_point_t goal_list[MAX_GOAL_LIST_SIZE];
static int annotations[MAX_GOAL_LIST_SIZE];
static int goal_list_index = 0;
static int goal_list_size = 0;
static carmen_obstacle_distance_mapper_message *current_map = NULL;
static carmen_behavior_selector_state_t current_state = BEHAVIOR_SELECTOR_FOLLOWING_LANE;
static carmen_behavior_selector_goal_source_t current_goal_source = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
static double change_goal_distance = 8.0; // @@@ Alberto: acho que nao usa... deletar?
static carmen_behavior_selector_algorithm_t following_lane_planner;
static carmen_behavior_selector_algorithm_t parking_planner;
static double distance_to_remove_annotation_goal = 3.0;

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

extern carmen_rddf_annotation_message last_rddf_annotation_message;

extern bool wait_start_moving;

// Information on front moving obstacle.
struct moving_obstacle_ahead_t
{
	bool found;

	carmen_ackerman_traj_point_t pose;

	double timestamp;
};

static struct moving_obstacle_ahead_t moving_obstacle_ahead = {false, {0, 0, 0, 0, 0}, 0};

bool is_moving_obstacle_ahead()
{
	return moving_obstacle_ahead.found;
}

static void moving_obstacle_ahead_reset(void)
{
	moving_obstacle_ahead.found = false;
}

static void moving_obstacle_ahead_set_pose(const carmen_ackerman_traj_point_t &pose)
{
	moving_obstacle_ahead.found = true;
	moving_obstacle_ahead.pose = pose;
	moving_obstacle_ahead.timestamp = carmen_get_time();
}


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
change_state(int rddf_annotation)
{
	if (current_goal_source == CARMEN_BEHAVIOR_SELECTOR_USER_GOAL)
		return;

	switch(rddf_annotation)
	{
	case RDDF_ANNOTATION_TYPE_NONE:
		current_state = BEHAVIOR_SELECTOR_FOLLOWING_LANE;
		break;

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
add_goal_to_goal_list(int &goal_index, carmen_ackerman_traj_point_t &current_goal, int rddf_pose_index, carmen_rddf_road_profile_message *rddf)
{
	goal_list[goal_index] = rddf->poses[rddf_pose_index];
	annotations[goal_index] = rddf->annotations[rddf_pose_index];
	current_goal = rddf->poses[rddf_pose_index];
	goal_index++;
}


void
add_goal_to_goal_list(int &goal_index, carmen_ackerman_traj_point_t &current_goal, int rddf_pose_index, carmen_rddf_road_profile_message *rddf,
		double displacement)
{
	carmen_ackerman_traj_point_t new_car_traj_point = rddf->poses[rddf_pose_index];
	carmen_point_t new_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&new_car_traj_point, displacement);
	new_car_traj_point.x = new_car_pose.x;
	new_car_traj_point.y = new_car_pose.y;

	goal_list[goal_index] = new_car_traj_point;
	annotations[goal_index] = rddf->annotations[rddf_pose_index];
	current_goal = new_car_traj_point;
	goal_index++;
}


int
get_parameters_for_filling_in_goal_list(int moving_object_in_front_index, int &last_obstacle_index, int &last_obstacle_free_waypoint_index,
		double &distance_from_car_to_rddf_point, double &distance_to_last_obstacle, double &distance_to_annotation,
		double &distance_to_last_obstacle_free_waypoint,
		carmen_rddf_road_profile_message *rddf, int rddf_pose_index, carmen_ackerman_traj_point_t current_goal, double circle_radius)
{
	int rddf_pose_hit_obstacle = trajectory_pose_hit_obstacle(rddf->poses[rddf_pose_index], circle_radius, current_map, &robot_config);

	if (rddf_pose_hit_obstacle || (moving_object_in_front_index != -1))
		last_obstacle_index = rddf_pose_index;
	else
		last_obstacle_free_waypoint_index = rddf_pose_index;

	if (rddf_pose_index == 0)
		last_obstacle_free_waypoint_index = 0;

	distance_from_car_to_rddf_point = carmen_distance_ackerman_traj(&current_goal, &rddf->poses[rddf_pose_index]);

	if (last_obstacle_index != -1)
		distance_to_last_obstacle = carmen_distance_ackerman_traj(&rddf->poses[last_obstacle_index], &rddf->poses[rddf_pose_index]);

	distance_to_annotation = carmen_distance_ackerman_traj(&current_goal, &rddf->poses[rddf_pose_index]) -
			(robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels);
	distance_to_last_obstacle_free_waypoint = carmen_distance_ackerman_traj(&current_goal, &rddf->poses[last_obstacle_free_waypoint_index]);

	return (rddf_pose_hit_obstacle);
}


int
move_goal_back_according_to_car_v(int last_obstacle_free_waypoint_index, carmen_rddf_road_profile_message *rddf,
		carmen_ackerman_traj_point_t robot_pose)
{
	int i;

	if (robot_pose.v > 4.0)
		return (last_obstacle_free_waypoint_index);
	else
	{
//		printf("entrei\n");
		double safe_distance = robot_config.distance_between_front_and_rear_axles +
				robot_config.distance_between_front_car_and_front_wheels + 4.0;
		for (i = last_obstacle_free_waypoint_index; i > 0; i--)
		{
			double distance = carmen_udatmo_front_obstacle_distance(&(rddf->poses[i])); //DIST2D(robot_pose, rddf->poses[i]);
//			printf("  i %d, d %lf, sd %lf\n", i, distance, safe_distance);
			if (distance > safe_distance)
			{
//				printf("# i %d, d %lf, sd %lf\n", i, distance, safe_distance);
				break;
			}
		}
	}
	if (i > 0)
		return (i);
	else
		return (last_obstacle_free_waypoint_index);
}


bool
red_traffic_light_ahead()
{
	static double last_red_timestamp = 0.0;
//	int nearest_annotation_index = -1;
//	double distance_to_nearest_annotation = 1000.0;
	for (int i = 0; i < last_rddf_annotation_message.num_annotations; i++)
	{
//		double distance_to_annotation = DIST2D_P(&last_rddf_annotation_message.annotations[i].annotation_point, current_robot_pose_v_and_phi);
		if ((last_rddf_annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT) &&
			(last_rddf_annotation_message.annotations[i].annotation_code == RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN)) //&&
//			 (distance_to_annotation < distance_to_nearest_annotation))
		{
			return (false);
		}
		else
		{
			last_red_timestamp = carmen_get_time();
		}
	}

	if (carmen_get_time() - last_red_timestamp < 2.0)
		return (true);

	return (false);
}


bool
recent_moving_object_near_this_rddf_pose(carmen_ackerman_traj_point_t pose)
{
	return (!moving_obstacle_ahead.found && DIST2D(pose, moving_obstacle_ahead.pose) < 2.0);
}


int
behaviour_selector_fill_goal_list(carmen_rddf_road_profile_message *rddf, double timestamp)
{
	double distance_to_last_obstacle = 10000.0;
	int last_obstacle_index = -1;

	int last_obstacle_free_waypoint_index = -1;

	goal_list_index = 0;
	goal_list_size = 0;

	if (rddf == NULL)
		return (0);

	carmen_udatmo_moving_obstacles_message *moving_obstacles = carmen_udatmo_detect_moving_obstacles();
	int front_obstacle_rddf_index = moving_obstacles->obstacles[0].rddf_index;
	moving_obstacle_ahead_reset();

	int goal_index = 0;
	carmen_ackerman_traj_point_t current_goal = robot_pose;
//	virtual_laser_message.num_positions = 0;
	double circle_radius = (robot_config.width - 0.0) / 2.0; // @@@ Alberto: metade da largura do carro + um espacco de guarda (ver valor certo)
	for (int rddf_pose_index = 0; rddf_pose_index < rddf->number_of_poses && goal_index < MAX_GOAL_LIST_SIZE; rddf_pose_index++)
	{
		double distance_from_car_to_rddf_point, distance_to_annotation, distance_to_last_obstacle_free_waypoint;
		int rddf_pose_hit_obstacle = -1;

		int moving_object_in_front_index = (front_obstacle_rddf_index == rddf_pose_index ? rddf_pose_index : -1);
		rddf_pose_hit_obstacle = get_parameters_for_filling_in_goal_list(moving_object_in_front_index, last_obstacle_index,
				last_obstacle_free_waypoint_index, distance_from_car_to_rddf_point, distance_to_last_obstacle, distance_to_annotation,
				distance_to_last_obstacle_free_waypoint, rddf, rddf_pose_index, current_goal, circle_radius);

		if (goal_index == 0 && moving_object_in_front_index != -1) // -> Adiciona um waypoint se a posicao atual colide com um objeto movel.
		{
			CARMEN_LOG(trace, "Set goal to moving obstacle");
			int moving_obstacle_waypoint = move_goal_back_according_to_car_v(last_obstacle_free_waypoint_index, rddf, robot_pose);
			add_goal_to_goal_list(goal_index, current_goal, moving_obstacle_waypoint, rddf);
			moving_obstacle_ahead_set_pose(rddf->poses[rddf_pose_index]);
			break;
		}
		else if (goal_index == 0 && rddf_pose_hit_obstacle && recent_moving_object_near_this_rddf_pose(rddf->poses[rddf_pose_index]))
		{
			CARMEN_LOG(trace, "Moving obstacle stopped");
			add_goal_to_goal_list(goal_index, current_goal, last_obstacle_free_waypoint_index, rddf,
					-(robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels));
			carmen_vector_3D_t annotation_point;
			annotation_point.x = rddf->poses[rddf_pose_index].x;
			annotation_point.y = rddf->poses[rddf_pose_index].y;
			annotation_point.z = 0.0;
			publish_dynamic_annotation(annotation_point, rddf->poses[rddf_pose_index].theta, (char *) "",
									   RDDF_ANNOTATION_TYPE_DYNAMIC, RDDF_ANNOTATION_CODE_DYNAMIC_STOP, timestamp);
			break;
		}
//		else if (rddf_pose_hit_obstacle)
//		{
//			add_goal_to_goal_list(goal_index, current_goal, last_obstacle_free_waypoint_index, rddf);
//			break;
//		}
		else if (((distance_from_car_to_rddf_point >= distance_between_waypoints) && // -> Adiciona um waypoint na posicao atual se ela esta numa distancia apropriada
				  (distance_to_last_obstacle >= 15.0) && // e se ela esta pelo menos 15.0 metros aa frente de um obstaculo
				  !rddf_pose_hit_obstacle)) // e se ela nao colide com um obstaculo.
		{
			add_goal_to_goal_list(goal_index, current_goal, rddf_pose_index, rddf);
		}
		else if ((((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BUMP) || // -> Adiciona um waypoint na posicao atual se ela contem uma das anotacoes especificadas
				   (rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BARRIER) ||
				   (rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK)) &&
				  (distance_to_annotation > distance_to_remove_annotation_goal) && // e se ela esta a uma distancia apropriada da anotacao
				  !rddf_pose_hit_obstacle)) // e se ela nao colide com um obstaculo.
		{
			add_goal_to_goal_list(goal_index, current_goal, rddf_pose_index, rddf,
					-(robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels));
		}
		else if ((((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_STOP) && !wait_start_moving) || // -> Adiciona um waypoint na posicao atual se ela contem uma das anotacoes especificadas
				  ((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP) && !wait_start_moving && red_traffic_light_ahead())) &&
				  !rddf_pose_hit_obstacle) // e se ela nao colide com um obstaculo.
		{
			add_goal_to_goal_list(goal_index, current_goal, rddf_pose_index, rddf,
					-(robot_config.distance_between_front_and_rear_axles + robot_config.distance_between_front_car_and_front_wheels));
			break;
		}
		else if (((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BUMP) || // -> Adiciona um waypoint na ultima posicao livre se a posicao atual contem uma das anotacoes especificadas
				  (rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_BARRIER) ||
				  ((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_STOP) && !wait_start_moving) ||
				  ((rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT_STOP) && !wait_start_moving) ||
				  (rddf->annotations[rddf_pose_index] == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK)) &&
				 (distance_to_last_obstacle_free_waypoint > 1.5) && // e se ela esta a mais de 1.5 metros da ultima posicao livre de obstaculo
				 rddf_pose_hit_obstacle) // e se ela colidiu com obstaculo.
		{	// Ou seja, se a anotacao estiver em cima de um obstaculo, adiciona um waypoint na posicao anterior mais proxima da anotacao que estiver livre.
			add_goal_to_goal_list(goal_index, current_goal, last_obstacle_free_waypoint_index, rddf);
		}
		else if ((rddf_pose_hit_obstacle == 2) && (goal_index == 0) && (rddf_pose_index > 10)) // Goal ideal esta fora do mapa
		{
			add_goal_to_goal_list(goal_index, current_goal, last_obstacle_free_waypoint_index, rddf);
			break;
		}
	}

//	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, timestamp);

	if (goal_index == 0)
	{
		goal_list[goal_index] = rddf->poses[rddf->number_of_poses - 1];
		goal_index++;
	}

	goal_list_size = goal_index;

	if (goal_list_index < goal_list_size)
	{
		change_state(annotations[goal_list_index]);
		return (1);
	}
	else
		return (0);
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


void
behavior_selector_update_map(carmen_obstacle_distance_mapper_message *map)
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

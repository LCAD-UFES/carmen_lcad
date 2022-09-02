#include "navigator_astar.hpp"
#include <iostream>

#include "navigator_ackerman_ipc.h"

#include "MessageControl.hpp"

#include <carmen/grid_mapping.h>
#include <prob_map.h>
#include <carmen/motion_planner_interface.h>
#include <carmen/rddf_interface.h>
#include <carmen/route_planner_interface.h>


static carmen_robot_ackerman_config_t robot_config;
static carmen_navigator_config_t nav_config;

static carmen_behavior_selector_algorithm_t current_algorithm = CARMEN_BEHAVIOR_SELECTOR_A_STAR;
carmen_behavior_selector_task_t current_task = BEHAVIOR_SELECTOR_PARK;
int steering_model = 1;
MessageControl messageControl;


#define LANE_LEFT_WIDTH 	(4.0 / 2.0) // 2.4
#define LANE_RIGHT_WIDTH 	((4.0 / 2.0) - 0.5) // 2.4
#define NUM_LANES	1


void
add_lanes(carmen_route_planner_road_network_message &route_planner_road_network_message)
{
    route_planner_road_network_message.number_of_nearby_lanes = NUM_LANES;
    route_planner_road_network_message.nearby_lanes_ids =  (int *) malloc(route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_indexes = (int *) malloc(route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_sizes = (int *) malloc(route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_size = NUM_LANES *
    		(route_planner_road_network_message.number_of_poses_back + route_planner_road_network_message.number_of_poses - 1);	// a primeira pose do poses e poses back eh igual
    route_planner_road_network_message.nearby_lanes = (carmen_robot_and_trailers_traj_point_t *) malloc(route_planner_road_network_message.nearby_lanes_size * sizeof(carmen_robot_and_trailers_traj_point_t));
    route_planner_road_network_message.traffic_restrictions =  (int *) malloc(route_planner_road_network_message.nearby_lanes_size * sizeof(int));

    // Coloca o rddf como a lane 0, isto eh, a rota escolhida
    route_planner_road_network_message.nearby_lanes_ids[0] = 0;
    route_planner_road_network_message.nearby_lanes_indexes[0] = 0;
    route_planner_road_network_message.nearby_lanes_sizes[0] = route_planner_road_network_message.number_of_poses_back + route_planner_road_network_message.number_of_poses - 1; // a primeira pose do poses e poses back eh igual
    int pose_i = 0;
    for (int i = route_planner_road_network_message.number_of_poses_back - 1; i > 0; i--)
	{
    	route_planner_road_network_message.nearby_lanes[pose_i] = route_planner_road_network_message.poses_back[i];
    	route_planner_road_network_message.traffic_restrictions[pose_i] = ROUTE_PLANNER_SET_LANE_LEFT_WIDTH(0, LANE_LEFT_WIDTH);
    	route_planner_road_network_message.traffic_restrictions[pose_i] = ROUTE_PLANNER_SET_LANE_RIGHT_WIDTH(route_planner_road_network_message.traffic_restrictions[pose_i], LANE_RIGHT_WIDTH);
    	pose_i++;
	}
    for (int i = 0; i < route_planner_road_network_message.number_of_poses; i++)
	{
    	route_planner_road_network_message.nearby_lanes[pose_i] = route_planner_road_network_message.poses[i];
    	route_planner_road_network_message.traffic_restrictions[pose_i] = ROUTE_PLANNER_SET_LANE_LEFT_WIDTH(0, LANE_LEFT_WIDTH);
    	route_planner_road_network_message.traffic_restrictions[pose_i] = ROUTE_PLANNER_SET_LANE_RIGHT_WIDTH(route_planner_road_network_message.traffic_restrictions[pose_i], LANE_RIGHT_WIDTH);
    	pose_i++;
	}
}


void
free_lanes(carmen_route_planner_road_network_message route_planner_road_network_message)
{
    free(route_planner_road_network_message.nearby_lanes_indexes);
    free(route_planner_road_network_message.nearby_lanes_sizes);
    free(route_planner_road_network_message.nearby_lanes_ids);
    free(route_planner_road_network_message.nearby_lanes);
    free(route_planner_road_network_message.traffic_restrictions);
}


int
get_index_of_nearest_pose_in_path(carmen_robot_and_trailers_traj_point_t *path, carmen_point_t globalpos, int path_length)
{
	int nearest_pose_index = 0;
	double min_dist = DIST2D(path[nearest_pose_index], globalpos);
	for (int i = 1; i < path_length; i++)
	{
		double distance = DIST2D(path[i], globalpos);
		if (distance < min_dist)
		{
			min_dist = distance;
			nearest_pose_index = i;
		}
	}

	return (nearest_pose_index);
}


carmen_robot_and_trailers_traj_point_t *
get_poses_back(carmen_robot_and_trailers_traj_point_t *path, int nearest_pose_index)
{
	carmen_robot_and_trailers_traj_point_t *poses_back = (carmen_robot_and_trailers_traj_point_t *) malloc((nearest_pose_index + 1) * sizeof(carmen_robot_and_trailers_traj_point_t));
	for (int i = 0; i < (nearest_pose_index + 1); i++)
		poses_back[i] = path[nearest_pose_index - i];

	return (poses_back);
}


void
publish_plan(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	carmen_planner_status_t status;
	messageControl.carmen_planner_ackerman_get_status(&status);
	if (status.path.length > 0)
	{
		int *annotations = (int *) calloc (status.path.length, sizeof(int));
		int *annotations_codes = (int *) calloc (status.path.length, sizeof(int));
		for (int i = 0; i < status.path.length; i++)
		{
			annotations[i] = RDDF_ANNOTATION_TYPE_NONE;
			annotations_codes[i] = RDDF_ANNOTATION_CODE_NONE;
		}

		int nearest_pose_index = get_index_of_nearest_pose_in_path(status.path.points, globalpos_message->globalpos, status.path.length);

	    carmen_route_planner_road_network_message route_planner_road_network_message;
	    route_planner_road_network_message.poses = &(status.path.points[nearest_pose_index]);
	    route_planner_road_network_message.poses_back = get_poses_back(status.path.points, nearest_pose_index);
	    route_planner_road_network_message.number_of_poses = status.path.length - nearest_pose_index;
	    route_planner_road_network_message.number_of_poses_back = nearest_pose_index + 1;	// tem que ter pelo menos uma pose_back que eh igual aa primeira poses
	    route_planner_road_network_message.annotations = annotations;
	    route_planner_road_network_message.annotations_codes = annotations_codes;

	    add_lanes(route_planner_road_network_message);

	    route_planner_road_network_message.timestamp = globalpos_message->timestamp;
	    route_planner_road_network_message.host = carmen_get_host();

	    carmen_route_planner_publish_road_network_message(&route_planner_road_network_message);
	    free_lanes(route_planner_road_network_message);

	    free(annotations);
	    free(annotations_codes);
	    free(route_planner_road_network_message.poses_back);
		free(status.path.points);
	}

}


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
//	if (current_algorithm != CARMEN_BEHAVIOR_SELECTOR_A_STAR)
//		return;
	IPC_RETURN_TYPE err = IPC_OK;
	static int roundValue = 2;
	static carmen_robot_and_trailers_traj_point_t robot_position;

	robot_position.x = round(msg->globalpos.x * roundValue) / roundValue;
	robot_position.y = round(msg->globalpos.y * roundValue) / roundValue;
	robot_position.theta = round(msg->globalpos.theta * roundValue) / roundValue;
	robot_position.trailer_theta[0] = round(msg->beta * roundValue) / roundValue;
	robot_position.v = msg->v;
	robot_position.phi = msg->phi;


	if (messageControl.carmen_planner_ackerman_update_robot(&robot_position, &robot_config) == 0)
		return;

	//publica caminho
	carmen_planner_status_t status;
	messageControl.carmen_planner_ackerman_get_status(&status);

	//Mensagem que faz andar
	if (status.path.length > 0)
	{
		publish_plan(msg);

		carmen_motion_planner_publish_path_message(status.path.points, status.path.length, current_algorithm);


		IPC_RETURN_TYPE err;
		static int firsttime = 1;
		static carmen_navigator_ackerman_astar_goal_list_message goal_list_msg;
		if (firsttime)
		{
			err = IPC_defineMsg(CARMEN_ASTAR_GOAL_LIST_NAME, IPC_VARIABLE_LENGTH, CARMEN_ASTAR_GOAL_LIST_FMT);
			carmen_test_ipc_exit(err, "Could not define message", CARMEN_ASTAR_GOAL_LIST_NAME);
			goal_list_msg.host = carmen_get_host();
			firsttime = 0;
		}
		goal_list_msg.goal_list = status.path.points;
		goal_list_msg.size = status.path.length;
		goal_list_msg.timestamp = carmen_get_time();

		err = IPC_publishData(CARMEN_ASTAR_GOAL_LIST_NAME, &goal_list_msg);//todo valgrind encontra problema de valor nao inicializado aqui
		carmen_test_ipc(err, "Could not publish", CARMEN_ASTAR_GOAL_LIST_NAME);
	}

	//Mensagem que exibe o caminho na tela
	if (status.path.length > 0)
	{
		static carmen_navigator_ackerman_plan_tree_message plan_tree_msg;
		static bool firstTime = true;

		if (firstTime == true)
		{
			err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME, IPC_VARIABLE_LENGTH,
					CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_FMT);
			carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME);

			plan_tree_msg.host = carmen_get_host();
			firstTime = false;
		}
		if (status.path.length > CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE)
		{	// Ver tipo carmen_navigator_ackerman_plan_tree_message
			printf("Error: status.path.length > %d\n", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE);
			return;
		}
		plan_tree_msg.num_path = 1;
		memcpy(plan_tree_msg.paths[0], status.path.points, sizeof(carmen_robot_and_trailers_traj_point_t) * status.path.length);
		plan_tree_msg.path_size[0] = status.path.length;
		err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME, &plan_tree_msg);

		carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME);
	}

}


void
path_goals_and_annotations_message_handler(carmen_behavior_selector_path_goals_and_annotations_message *msg)
{

	if ((msg->goal_list_size <= 0) || !msg->goal_list)
		return;

	messageControl.carmen_planner_ackerman_update_goal(msg->goal_list);
}


static void
state_handler(carmen_behavior_selector_state_message *msg)
{
//	static carmen_behavior_selector_goal_source_t goal_source = CARMEN_BEHAVIOR_SELECTOR_USER_GOAL;
	current_algorithm = msg->algorithm;
	current_task = msg->task;

//	if (goal_source != msg->goal_source)
//	{
//		goal_source = msg->goal_source;
//		carmen_robot_and_trailer_traj_point_t point;
//		point.x = -1;
//		point.y = -1;
//		point.theta = -1;
//		messageControl.carmen_planner_ackerman_update_goal(&point);
//	}
}


static void
map_server_compact_cost_map_message_handler(carmen_map_server_compact_cost_map_message *message)
{
	static carmen_map_t cost_map;
	static carmen_compact_map_t *compact_cost_map = NULL;
	if (compact_cost_map == NULL)
	{
		carmen_grid_mapping_create_new_map(&cost_map, message->config.x_size, message->config.y_size, message->config.resolution, 'm');
		memset(cost_map.complete_map, 0, cost_map.config.x_size * cost_map.config.y_size * sizeof(double));

		compact_cost_map = (carmen_compact_map_t *) (calloc(1, sizeof(carmen_compact_map_t)));
		carmen_cpy_compact_map_message_to_compact_map(compact_cost_map, message);
		carmen_prob_models_uncompress_compact_map(&cost_map, compact_cost_map);
	}
	else
	{
		carmen_prob_models_clear_carmen_map_using_compact_map(&cost_map, compact_cost_map, 0.0);
		carmen_prob_models_free_compact_map(compact_cost_map);
		carmen_cpy_compact_map_message_to_compact_map(compact_cost_map, message);
		carmen_prob_models_uncompress_compact_map(&cost_map, compact_cost_map);
	}

	messageControl.carmen_planner_ackerman_set_cost_map(&cost_map);
	cost_map.config = message->config;
}


void
carmen_rddf_play_end_point_message_handler(carmen_rddf_end_point_message *rddf_end_point_message)
{
	carmen_robot_and_trailers_traj_point_t point =
	{
		rddf_end_point_message->point.x,
		rddf_end_point_message->point.y,
		rddf_end_point_message->point.theta,
		rddf_end_point_message->point.num_trailers,
		rddf_end_point_message->point.trailer_theta[0],
		0.0,
		0.0
	};
	messageControl.carmen_planner_ackerman_update_goal(&point);
}


static void
navigator_shutdown(int signal)
{
	static int done = 0;

	if (!done)
	{
		carmen_ipc_disconnect();
		printf("Disconnected from IPC. signal = %d\n", signal);
		done = 1;
	}
	exit(0);
}


static void
read_parameters(int argc, char **argv)
{
	int num_items;
	carmen_navigator_ackerman_astar_t astar_config;


	carmen_param_t param_list[] = 
	{
			{(char *)"robot",				(char *)"max_velocity", CARMEN_PARAM_DOUBLE, &robot_config.max_v, 1, NULL},//todo add max_v and max_phi in carmen.ini
			{(char *)"robot",				(char *)"max_steering_angle", CARMEN_PARAM_DOUBLE, &robot_config.max_phi, 1, NULL},
			{(char *)"robot",				(char *)"min_approach_dist", CARMEN_PARAM_DOUBLE, &robot_config.approach_dist, 1, NULL},
			{(char *)"robot",				(char *)"min_side_dist", CARMEN_PARAM_DOUBLE, &robot_config.side_dist, 1, NULL},
			{(char *)"robot",				(char *)"length", CARMEN_PARAM_DOUBLE, &robot_config.length, 0, NULL},
			{(char *)"robot",				(char *)"width", CARMEN_PARAM_DOUBLE, &robot_config.width, 0, NULL},
			{(char *)"robot",				(char *)"maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward, 1, NULL},
			{(char *)"robot",				(char *)"reaction_time", CARMEN_PARAM_DOUBLE,	&robot_config.reaction_time, 0, NULL},
			{(char *)"robot",				(char *)"distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 1, NULL},
			{(char *)"robot",				(char *)"maximum_steering_command_rate", CARMEN_PARAM_DOUBLE, &robot_config.maximum_steering_command_rate, 1, NULL},
			{(char *)"robot",				(char *)"distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
			{(char *)"navigator",			(char *)"goal_size", CARMEN_PARAM_DOUBLE, &nav_config.goal_size, 1, NULL},
			{(char *)"navigator",			(char *)"waypoint_tolerance", CARMEN_PARAM_DOUBLE, &nav_config.waypoint_tolerance, 1, NULL},
			{(char *)"navigator",			(char *)"goal_theta_tolerance", CARMEN_PARAM_DOUBLE, &nav_config.goal_theta_tolerance, 1, NULL},
			{(char *)"navigator",			(char *)"map_update_radius", CARMEN_PARAM_DOUBLE,	&nav_config.map_update_radius, 1, NULL},
			{(char *)"navigator",			(char *)"map_update_num_laser_beams", CARMEN_PARAM_INT, &nav_config.num_lasers_to_use, 1, NULL},
			{(char *)"navigator",			(char *)"map_update_obstacles", CARMEN_PARAM_ONOFF, &nav_config.map_update_obstacles, 1, NULL},
			{(char *)"navigator",			(char *)"map_update_freespace", CARMEN_PARAM_ONOFF, &nav_config.map_update_freespace, 1, NULL},
			{(char *)"navigator",			(char *)"replan_frequency", CARMEN_PARAM_DOUBLE, &nav_config.replan_frequency, 1, NULL},
			{(char *)"navigator",			(char *)"smooth_path", CARMEN_PARAM_ONOFF, &nav_config.smooth_path, 1, NULL},
			{(char *)"navigator",			(char *)"dont_integrate_odometry", CARMEN_PARAM_ONOFF, &nav_config.dont_integrate_odometry, 1, NULL},
			{(char *)"navigator",			(char *)"plan_to_nearest_free_point", CARMEN_PARAM_ONOFF,	&nav_config.plan_to_nearest_free_point, 1, NULL},
			{(char *)"navigator_astar",		(char *)"path_interval", CARMEN_PARAM_DOUBLE, &astar_config.path_interval, 1, NULL},
			{(char *)"navigator_astar",		(char *)"state_map_resolution", CARMEN_PARAM_INT, &astar_config.state_map_resolution, 1, NULL},
			{(char *)"navigator_astar",		(char *)"state_map_theta_resolution", CARMEN_PARAM_INT, &astar_config.state_map_theta_resolution, 1, NULL},
			{(char *)"navigator_astar",		(char *)"precomputed_cost_size", CARMEN_PARAM_INT, &astar_config.precomputed_cost_size, 1, NULL},
			{(char *)"navigator_astar",		(char *)"precomputed_cost_file_name", CARMEN_PARAM_STRING, &astar_config.precomputed_cost_file_name, 1, NULL},
			{(char *)"navigator_astar",		(char *)"use_rs", CARMEN_PARAM_ONOFF, &astar_config.use_rs, 1, NULL},
			{(char *)"navigator_astar",		(char *)"smooth_path", CARMEN_PARAM_ONOFF, &astar_config.smooth_path, 1, NULL},
			{(char *)"navigator_astar",		(char *)"onroad_max_plan_time", CARMEN_PARAM_DOUBLE, &astar_config.onroad_max_plan_time, 1, NULL},
			{(char *)"navigator_astar",		(char *)"robot_fat_space", CARMEN_PARAM_DOUBLE, &astar_config.robot_fat_space, 1, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	messageControl.initAstarParans(astar_config);
}


int 
main(int argc, char **argv)
{

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	if (carmen_navigator_ackerman_initialize_ipc() < 0)
		carmen_die("Error: could not connect to IPC Server\n");

	signal(SIGINT, navigator_shutdown);

	carmen_route_planner_define_messages();

	carmen_map_server_subscribe_compact_cost_map(NULL, (carmen_handler_t) map_server_compact_cost_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_path_goals_and_annotations_message(NULL, (carmen_handler_t) path_goals_and_annotations_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) state_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t)localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_end_point_message(NULL, (carmen_handler_t) carmen_rddf_play_end_point_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return 0;
}

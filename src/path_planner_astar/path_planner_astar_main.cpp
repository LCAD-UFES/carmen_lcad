#include "path_planner_astar.h"

carmen_robot_ackerman_config_t robot_config;
carmen_path_planner_astar_t astar_config;
static carmen_navigator_config_t nav_config;

static carmen_obstacle_distance_mapper_compact_map_message *compact_distance_map = NULL;
static carmen_obstacle_distance_mapper_map_message distance_map_struct;
carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map = NULL;
carmen_map_p map_occupancy = NULL;

nonholonomic_heuristic_cost_p ***nonholonomic_heuristic_cost_map;
int use_nonholonomic_heuristic_cost_map = 1;

carmen_point_t *final_goal = NULL;
int final_goal_received = 0;
int path_sended = 0;

int grid_state_map_x_size;
int grid_state_map_y_size;

carmen_route_planner_road_network_message route_planner_road_network_message;
offroad_planner_plan_t plan_path_poses;
std::vector<carmen_ackerman_traj_point_t> astar_path;

clock_t r_time;

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_plan(offroad_planner_path_t path, carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	int *annotations = (int *) calloc (path.length, sizeof(int));
	int *annotations_codes = (int *) calloc (path.length, sizeof(int));
	for (int i = 0; i < path.length; i++)
	{
		annotations[i] = RDDF_ANNOTATION_TYPE_NONE;
		annotations_codes[i] = RDDF_ANNOTATION_CODE_NONE;
	}
	int nearest_pose_index = get_index_of_nearest_pose_in_path(path.points, globalpos_message->globalpos, path.length);
	carmen_ackerman_traj_point_t *path_copy = NULL;

	// Por alguma razão, o route_planner_road_network_message só funciona corretamente se for global
//	carmen_route_planner_road_network_message route_planner_road_network_message;

	route_planner_road_network_message.poses = &(path.points[nearest_pose_index]);
	route_planner_road_network_message.poses_back = get_poses_back(path.points, nearest_pose_index);
	route_planner_road_network_message.number_of_poses = path.length - nearest_pose_index;
	route_planner_road_network_message.number_of_poses_back = nearest_pose_index + 1;	// tem que ter pelo menos uma pose_back que eh igual aa primeira poses
//	route_planner_road_network_message.number_of_poses_back = 1;
	route_planner_road_network_message.annotations = annotations;
	route_planner_road_network_message.annotations_codes = annotations_codes;
	add_lanes(route_planner_road_network_message, path_copy);
	route_planner_road_network_message.timestamp = globalpos_message->timestamp;
	route_planner_road_network_message.host = carmen_get_host();
//	first_published = 0;

	// Changes in route planner message //////////////////////////////////////////////////////////////////////
    static int *nearby_lanes_node_ids = NULL;	// Size == nearby_lanes_size. Ids dos nós (poses) de todas as lanes.

	static int *nearby_lanes_merges_indexes = NULL;	// Size == number_of_nearby_lanes. O ponto em nearby_lanes_merges onde começam os merges de cada lane.
	static int *nearby_lanes_merges_sizes = NULL;		// Size == number_of_nearby_lanes. O número de merges de cada lane.

	static int *nearby_lanes_forks_indexes = NULL;	// Size == number_of_nearby_lanes. O ponto em nearby_lanes_forks onde começam os forks de cada lane.
	static int *nearby_lanes_forks_sizes = NULL;		// Size == number_of_nearby_lanes. O número de forks de cada lane.

	static int *nearby_lanes_crossroads_indexes = NULL;
	static int *nearby_lanes_crossroads_sizes = NULL;

    nearby_lanes_node_ids = (int *) realloc(nearby_lanes_node_ids, route_planner_road_network_message.nearby_lanes_size * sizeof(int));
    route_planner_road_network_message.nearby_lanes_node_ids = nearby_lanes_node_ids;

    nearby_lanes_merges_indexes = (int *) realloc(nearby_lanes_merges_indexes, route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_merges_indexes = nearby_lanes_merges_indexes;
    nearby_lanes_merges_sizes = (int *) realloc(nearby_lanes_merges_sizes, route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_merges_sizes = nearby_lanes_merges_sizes;

    nearby_lanes_forks_indexes = (int *) realloc(nearby_lanes_forks_indexes, route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_forks_indexes = nearby_lanes_forks_indexes;
    nearby_lanes_forks_sizes = (int *) realloc(nearby_lanes_forks_sizes, route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
    route_planner_road_network_message.nearby_lanes_forks_sizes = nearby_lanes_forks_sizes;

    nearby_lanes_crossroads_indexes = (int *) realloc(nearby_lanes_crossroads_indexes, route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
   route_planner_road_network_message.nearby_lanes_crossroads_indexes = nearby_lanes_crossroads_indexes;
   nearby_lanes_crossroads_sizes = (int *) realloc(nearby_lanes_crossroads_sizes, route_planner_road_network_message.number_of_nearby_lanes * sizeof(int));
   route_planner_road_network_message.nearby_lanes_crossroads_sizes = nearby_lanes_crossroads_sizes;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	carmen_route_planner_publish_road_network_message(&route_planner_road_network_message);

//    IPC_RETURN_TYPE err;
//    err = IPC_publishData(CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_NAME, &route_planner_road_network_message);
//    carmen_test_ipc_exit(err, "Could not publish", CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_NAME);

	/*
	int i;
	time_count.reset();
	for (i = 0; i < route_planner_road_network_message.number_of_poses; i++){
//		printf("route_planner print %f %f %f %f %f\n", route_planner_road_network_message.poses[i].x, route_planner_road_network_message.poses[i].y, route_planner_road_network_message.poses[i].theta, route_planner_road_network_message.poses[i].v, route_planner_road_network_message.poses[i].phi);
//		printf("Collision check: %f \n",carmen_obstacle_avoider_car_distance_to_nearest_obstacle(route_planner_road_network_message.poses[i] ,distance_map));
		carmen_obstacle_avoider_car_distance_to_nearest_obstacle(route_planner_road_network_message.poses[i] ,distance_map);
	}
	printf("Collision check time is %f seconds for %d poses\n", time_count.get_since(), i+1);
*/

	free_lanes(route_planner_road_network_message);
	free(annotations);
	free(annotations_codes);
	free(route_planner_road_network_message.poses_back);
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	carmen_point_t robot_position = {msg->globalpos.x, msg->globalpos.y, msg->globalpos.theta};

	if (final_goal_received != 0)
	{
		r_time = clock();

		carmen_point_t goal_position = {final_goal->x, final_goal->y, final_goal->theta};
		if (!astar_path.empty())
			astar_path.clear();

		#if RUN_EXPERIMENT
			override_initial_and_goal_poses(robot_position, goal_position);
		#endif

		pose_node initial_pose, goal_pose;
		initial_pose = {robot_position.x, robot_position.y, robot_position.theta, Forward};
		goal_pose = {goal_position.x, goal_position.y, goal_position.theta, Forward};
		double *goal_distance_map = get_goal_distance_map(goal_position, obstacle_distance_grid_map);

		astar_path = carmen_path_planner_astar_search(&initial_pose, &goal_pose, obstacle_distance_grid_map, goal_distance_map, nonholonomic_heuristic_cost_map);

		if (astar_path.size() > 2)
		{
			printf("A-star search time = %f\n", (double)(clock() - r_time) / CLOCKS_PER_SEC);

			if (USE_SMOOTH)
			{
				smooth_rddf_using_conjugate_gradient(astar_path);
				printf("Full running time = %f\n", (double)(clock() - r_time) / CLOCKS_PER_SEC);
			}


			plan_path_poses = astar_mount_offroad_planner_plan(&robot_position, final_goal, astar_path);
			publish_plan(plan_path_poses.path, msg);
	}
//			for (int i = 0; i< astar_path.size(); i++)
//				printf("Path: %f %f %f %f %f\n", astar_path[i].x, astar_path[i].y, astar_path[i].theta, astar_path[i].v, astar_path[i].phi);


		free(goal_distance_map);

		final_goal_received = 0;
	}

	if(path_sended && DIST2D(robot_position, *route_planner_road_network_message.poses) > 4.0)
	{
		publish_plan(plan_path_poses.path, msg);
	}

	if(path_sended && DIST2D_P(&robot_position, final_goal) < 6.0 && abs(carmen_compute_abs_angular_distance(robot_position.theta, final_goal->theta)) < 0.2617995)
	{
		printf("Chegou ao destino\n");
		path_sended = 0;
	}
}


static void
carmen_obstacle_distance_mapper_compact_map_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	if (compact_distance_map == NULL)
	{
		obstacle_distance_grid_map = &distance_map_struct;
		carmen_obstacle_distance_mapper_create_new_map(obstacle_distance_grid_map, message->config, message->host, message->timestamp);
		compact_distance_map = (carmen_obstacle_distance_mapper_compact_map_message *) (calloc(1, sizeof(carmen_obstacle_distance_mapper_compact_map_message)));
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(obstacle_distance_grid_map, message);
	}
	else
	{
		carmen_obstacle_distance_mapper_clear_distance_map_message_using_compact_map(obstacle_distance_grid_map, compact_distance_map, DISTANCE_MAP_HUGE_DISTANCE);
		carmen_obstacle_distance_mapper_free_compact_distance_map(compact_distance_map);
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(obstacle_distance_grid_map, message);
	}

	obstacle_distance_grid_map->config = message->config;
}


void
carmen_path_planner_astar_mapper_handler(carmen_mapper_map_message *message)
{
	if (final_goal_received)
		return;

	static double last_time_stamp = 0.0;

	if (message->size <= 0)
		return;

	// @@@ Alberto: codigo adicionado para atualizar o mapa a uma taxa maxima de 10Hz
	if ((carmen_get_time() - last_time_stamp) > 0.1)
		last_time_stamp = carmen_get_time();
	else
		return;

	if (map_occupancy && (message->config.x_size != map_occupancy->config.x_size || message->config.y_size != map_occupancy->config.y_size))
	{
		carmen_map_destroy(&map_occupancy);
	}

	map_occupancy = copy_grid_mapping_to_map(map_occupancy, message);
}


static void
carmen_behaviour_selector_compact_lane_contents_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	if (obstacle_distance_grid_map)
		carmen_obstacle_distance_mapper_overwrite_distance_map_message_with_compact_distance_map(obstacle_distance_grid_map, message);
}


static void
carmen_rddf_play_end_point_message_handler(carmen_rddf_end_point_message *rddf_end_point_message)
{
	final_goal = &(rddf_end_point_message->point);
	final_goal_received = 1;
}


static void
offroad_planner_shutdown(int signal)
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
///////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
	carmen_obstacle_distance_mapper_subscribe_compact_map_message(NULL, (carmen_handler_t) carmen_obstacle_distance_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behaviour_selector_subscribe_compact_lane_contents_message(NULL, (carmen_handler_t) carmen_behaviour_selector_compact_lane_contents_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) (carmen_localize_ackerman_globalpos_message_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_end_point_message(NULL, (carmen_handler_t) (carmen_rddf_play_end_point_message_handler), CARMEN_SUBSCRIBE_LATEST);
	carmen_mapper_subscribe_map_message(NULL, (carmen_handler_t) carmen_path_planner_astar_mapper_handler, CARMEN_SUBSCRIBE_LATEST);

}


void
define_messages()
{
	carmen_route_planner_define_messages();
}


static void
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
//			{(char *)"robot", 				(char *) "max_steering_angle",CARMEN_PARAM_DOUBLE, &robot_config.max_phi,1, NULL},
			{(char *)"robot",				(char *)"max_velocity", CARMEN_PARAM_DOUBLE, &robot_config.max_v, 1, NULL},
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
			{(char *)"navigator",			(char *)"dont_integrate_odometry", CARMEN_PARAM_ONOFF, &nav_config.dont_integrate_odometry, 1, NULL},
			{(char *)"navigator",			(char *)"plan_to_nearest_free_point", CARMEN_PARAM_ONOFF,	&nav_config.plan_to_nearest_free_point, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"max_steering_angle", CARMEN_PARAM_DOUBLE, &robot_config.max_phi, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"state_map_resolution", CARMEN_PARAM_DOUBLE, &astar_config.state_map_resolution, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"state_map_theta_resolution", CARMEN_PARAM_INT, &astar_config.state_map_theta_resolution, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"precomputed_cost_size", CARMEN_PARAM_INT, &astar_config.precomputed_cost_size, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"precomputed_cost_theta_size", CARMEN_PARAM_INT, &astar_config.precomputed_cost_theta_size, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"precomputed_cost_resolution", CARMEN_PARAM_DOUBLE, &astar_config.precomputed_cost_resolution, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"precomputed_cost_file_name", CARMEN_PARAM_STRING, &astar_config.precomputed_cost_file_name, 1, NULL},
			{(char *)"path_planner_astar",	(char *)"use_matrix_cost_heuristic", CARMEN_PARAM_ONOFF, &astar_config.use_matrix_cost_heuristic, 1, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

//	if (0)
	if (astar_config.use_matrix_cost_heuristic)
		alloc_cost_map();
/*
	memset(&virtual_laser_message, 0, sizeof(carmen_mapper_virtual_laser_message));
	virtual_laser_message.positions = (carmen_position_t *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(carmen_position_t));
	virtual_laser_message.colors = (char *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(char));
	virtual_laser_message.host = carmen_get_host();
*/
}
///////////////////////////////////////////////////////////////////////////////////////////////////


void
print_parameters_astar()
{
	printf("Max steering angle = %f\n", robot_config.max_phi );
	printf("State map resolution = %f\n", astar_config.state_map_resolution );
	printf("State map theta resolution = %d\n", astar_config.state_map_theta_resolution );
	printf("Precomputed cost size = %d\n", astar_config.precomputed_cost_size );
	printf("Precomputed cost theta size = %d\n", astar_config.precomputed_cost_theta_size );
	printf("Precomputed cost resolution = %f\n", astar_config.precomputed_cost_resolution );
	printf("Precomputed cost file name = %s\n", astar_config.precomputed_cost_file_name );
	printf("Use matrix cost heuristic = %d\n", astar_config.use_matrix_cost_heuristic );

	if (SEND_MESSAGE_IN_PARTS)
		printf("SEND_MESSAGE_IN_PARTS ON, o path é enviado em partes\n");
	else
		printf("SEND_MESSAGE_IN_PARTS OFF, o path é enviado inteiro\n");
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	define_messages();
	subscribe_messages();
	print_parameters_astar();
	printf("Aguardando Final Goal\n");

	signal(SIGINT, offroad_planner_shutdown);

	carmen_ipc_dispatch();

	return (0);
}

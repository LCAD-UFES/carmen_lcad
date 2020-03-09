#include "path_planner_astar.h"
#define CLOSEST_CIRCLE_MIN_DIST 1.5
#define MIN_THETA_DIFF    0.0872665 // 5 0.0436332 // 2.5 // 0.261799 // 15 degrees
#define MIN_STEERING_DIFF 0.0872665
#define MIN_POS_DISTANCE  0.3 // the carmen grid map resolution

carmen_point_t *final_goal = NULL;
carmen_localize_ackerman_globalpos_message *current_globalpos_msg = NULL;
carmen_robot_ackerman_config_t robot_config;
carmen_obstacle_distance_mapper_map_message distance_map;


double
carmen_compute_abs_angular_distance(double theta_1, double theta_2)
{
	return (carmen_normalize_theta(abs(theta_1 - theta_2)));
}


double
carmen_compute_abs_steering_angular_distance(double phi_1, double phi_2)
{
	return (abs(phi_1 - phi_2));
}


state_node*
create_state_node(double x, double y, double theta, double v, double phi, double g, double h, state_node *parent)
{
	state_node *new_state = (state_node*) malloc(sizeof(state_node));
	new_state->state.x = x;
	new_state->state.y = y;
	new_state->state.theta = theta;
	new_state->state.v = v;
	new_state->state.phi = phi;
	new_state->f = g+h;
	new_state->g = g;
	new_state->h = h;
	new_state->parent = parent;

	return (new_state);
}


std::vector<state_node>
build_state_path(state_node *node)
{
	std::vector<state_node> state_path;
	int i = 0;
    while (node != NULL)
    {
        state_path.push_back(*node);

        node = node->parent;

        i++;
    }

    //printf("--- %u %d\n", (int) circle_path.size(), i);

    return(state_path);
}


int
state_node_exist(state_node *new_state, std::vector<state_node*> &closed_set)
{
	double distance = 0.0;
	double orientation_diff = 0.0;
	double steering_diff = 0.0;

	//printf ("%d\n", closed_set.size());

    for (unsigned int i = 0; i < closed_set.size(); i++)
    {
    	distance = DIST2D(new_state->state, closed_set[i]->state);
//
//    	if (distance < 0.5)
//    		printf ("State Exist! %lf %lf %lf %lf %lf %lf\n", distance, angular_distance);

    	orientation_diff = carmen_compute_abs_angular_distance(new_state->state.theta, closed_set[i]->state.theta);
    	steering_diff = carmen_compute_abs_steering_angular_distance(new_state->state.phi, closed_set[i]->state.phi);

    	if (distance < MIN_POS_DISTANCE && orientation_diff < MIN_THETA_DIFF && steering_diff < MIN_POS_DISTANCE && (new_state->state.v == closed_set[i]->state.v))
    	{
    		//printf ("State Exist! %lf %lf\n", distance, angular_distance);
    		return i+1;
    	}
    }

    return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
static void
pos_message_handler(carmen_point_t robot_pose)
{
	if (final_goal != NULL)
	{
		printf("Pos_message_handler: %lf %lf %lf\n", robot_pose.x, robot_pose.y, robot_pose.theta);
		compute_astar_path(&robot_pose, final_goal, robot_config , &distance_map);
		final_goal = NULL;
	}
}


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	current_globalpos_msg = msg;
	pos_message_handler(msg->globalpos/*, msg->v, msg->timestamp*/);
}


void
carmen_rddf_play_end_point_message_handler(carmen_rddf_end_point_message *rddf_end_point_message)
{
	final_goal = &(rddf_end_point_message->point);
	printf("carmen_rddf_play_end_point: %lf %lf %lf\n", rddf_end_point_message->point.x, rddf_end_point_message->point.y, rddf_end_point_message->point.theta);
//	printf ("Recebeu Goal!!!!!  %d\n", rddf_end_point_message->number_of_poses);
}


static void
carmen_obstacle_distance_mapper_compact_map_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	static carmen_obstacle_distance_mapper_compact_map_message *compact_distance_map = NULL;  // TODO is it possible to remove this line and the if

		if (compact_distance_map == NULL)
		{
			carmen_obstacle_distance_mapper_create_new_map(&distance_map, message->config, message->host, message->timestamp);
			compact_distance_map = (carmen_obstacle_distance_mapper_compact_map_message *) (calloc(1, sizeof(carmen_obstacle_distance_mapper_compact_map_message)));
			carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
			carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, message);
		}
		else
		{
			carmen_obstacle_distance_mapper_clear_distance_map_message_using_compact_map(&distance_map, compact_distance_map, DISTANCE_MAP_HUGE_DISTANCE);
			carmen_obstacle_distance_mapper_free_compact_distance_map(compact_distance_map);
			carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
			carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, message);
		}
//	printf("Mapa recebido!\n");
}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        printf("path_planner_astar_main: Disconnected.\n");
        exit(0);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Subscribes                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_rddf_play_subscribe_messages()
{
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_obstacle_distance_mapper_subscribe_compact_map_message(NULL, (carmen_handler_t) carmen_obstacle_distance_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_end_point_message(NULL, (carmen_handler_t) carmen_rddf_play_end_point_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


///////////////////////////////////////////////////////////////////////////////////////////////


double
obstacle_distance(double x, double y, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	if( NULL == distance_map)
		exit(1);

    carmen_point_t p;

    p.x = x;
    p.y = y;

    return (carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&p, distance_map));
}

bool
my_f_ordenation (state_node *a, state_node *b) { return (a->f > b->f); }


void
expand_state(state_node *current_state, state_node *goal_state, std::vector<state_node*> &closed_set, /*std::priority_queue<state_node*, std::vector<state_node*>, StateNodePtrComparator>*/ std::vector<state_node*> &open_set,
		carmen_robot_ackerman_config_t robot_config, carmen_obstacle_distance_mapper_map_message *distance_map)
{
    double target_phi, distance_traveled = 0.0;
	#define NUM_STEERING_ANGLES 3
    double steering_acceleration[NUM_STEERING_ANGLES] = {-0.25, 0.0, 0.25}; //TODO ler velocidade angular do volante do carmen.ini
    double target_v[3]   = {2.0, 0.0, -2.0};

    double add_x[3] = {-1.0, 0.0, 1.0};
    double add_y[3] = {-1.0, 0.0, 1.0};
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < NUM_STEERING_ANGLES; ++j)

        {
        	state_node *new_state = (state_node*) malloc(sizeof(state_node));

        	target_phi = carmen_clamp(-robot_config.max_phi, (current_state->state.phi + steering_acceleration[j]), robot_config.max_phi);

        	// Utilizando expansão não-holonomica
//        	new_state->state = carmen_libcarmodel_recalc_pos_ackerman(current_state->state, target_v[i], target_phi,
//        			0.25, &distance_traveled, DELTA_T, robot_config);

        	// Utilizando expansão holonomica
        	new_state->state.x = current_state->state.x + add_x[i];
        	new_state->state.y = current_state->state.y + add_y[j];
        	new_state->state.theta = current_state->state.theta;

        	new_state->parent = current_state;
        	// g = current até o start
        	//new_state->g = current_state->g + DIST2D(current_state->state, new_state->state);
        	new_state->g = current_state->g + DIST2D(current_state->state, new_state->state);
        	// h = current até o goal
        	//new_state->h = DIST2D(new_state->state, current_state->state) + current_state->h;
        	new_state->h = DIST2D(new_state->state, goal_state->state);
        	new_state->f = new_state->g + new_state->h;
//        	printf("obstacle distance = %lf\n", obstacle_distance(new_state->state.x, new_state->state.y, distance_map));
        	if ((obstacle_distance(new_state->state.x, new_state->state.y, distance_map) < 4.0*0.5) || state_node_exist(new_state, closed_set))   // TODO ler a margem de segurança do carmen.ini
        	{
        		free (new_state);
        	}
        	else
        	{
        		int indice = state_node_exist(new_state, open_set);

        		if(indice)
        		{
        			if(open_set[indice-1]->g > new_state->g)
        			{
        				open_set[indice-1]->g = new_state->g;
        				free(new_state);
        			}

        		}

        		else
        		{
        		open_set.push_back(new_state);
        		std::sort(open_set.begin(), open_set.end(), my_f_ordenation);
        		}

        	}
        }
    }
}


void
compute_astar_path(carmen_point_t *robot_pose, carmen_point_t *goal_pose, carmen_robot_ackerman_config_t robot_config,
		carmen_obstacle_distance_mapper_map_message *distance_map)
{
	state_node *start_state, *goal_state;

	start_state = create_state_node(robot_pose->x, robot_pose->y, robot_pose->theta, 0.0, 0.0, 0.0, DIST2D_P(robot_pose, goal_pose), NULL);
	goal_state = create_state_node(goal_pose->x, goal_pose->y, goal_pose->theta, 0.0, 0.0, DBL_MAX, DBL_MAX, NULL);
	std::vector<state_node> path;
//	std::priority_queue<state_node*, std::vector<state_node*>, StateNodePtrComparator> open_set;
	std::vector<state_node*> open_set;

	std::vector<state_node*> closed_set;
	open_set.push_back(start_state);
//	open_set.push(start_state);
	while (!open_set.empty())
	{
//		state_node *current_state = open_set.top();
//		open_set.pop();
		state_node *current_state = open_set.back();
		open_set.pop_back();


//		printf("--- State %lf %lf %lf %lf %lf %lf %lf\n", current_state->state.x, current_state->state.y, current_state->state.theta, current_state->state.v, current_state->state.phi, current_state->g, current_state->h);
//		printf("aa = %lf \n",DIST2D(current_state->state, goal_state->state));
		if (DIST2D(current_state->state, goal_state->state) < 0.5)
		{
			// Se chegou ao Goal, deleta toda a pilha do open_set e sai do loop.
			path = build_state_path(current_state->parent);
        	printf("Final obstacle distance = %lf\n", obstacle_distance(goal_state->state.x, goal_state->state.y, distance_map));
			std::reverse(path.begin(), path.end());
			std::vector<carmen_ackerman_traj_point_t> temp_rddf_poses_from_path;
			for(int i = 0; i < path.size(); i++)
			{
				temp_rddf_poses_from_path.push_back(path[i].state);
			}

			carmen_ackerman_traj_point_t *carmen_rddf_poses_from_path = &temp_rddf_poses_from_path[0];

			carmen_ackerman_traj_point_t last_pose = {.x = 0.0, .y = 0.0, .theta = 0.0, .v = 9.0, .phi=0.2};
			int annotations[2] = {1, 2};
			int annotation_codes[2] = {1, 2};
//			printf("pathsize = %d\n", path.size());
/*
			for (int i = 0; i < path.size(); i++)
			{
				printf("Poses do rddf: %f %f %d\n", carmen_rddf_poses_from_path[i].x, carmen_rddf_poses_from_path[i].y, i);
			//	printf("Poses do path: %f %f %d\n", path[i].state.x, path[i].state.y, i);
			}
*/
			carmen_rddf_publish_road_profile_message(
				carmen_rddf_poses_from_path,
				&last_pose,
				path.size(),
				1,
				annotations,
				annotation_codes);

			printf("Poses enviadas!\n");

			temp_rddf_poses_from_path.clear();

			while(!open_set.empty())
			{
//				state_node *tmp = open_set.top();
//				open_set.pop();
				state_node *tmp = open_set.back();
				open_set.pop_back();

				delete tmp;
			}

			break;

		}

		else
		{
		if (!state_node_exist(current_state, closed_set))
			expand_state(current_state, goal_state, closed_set, open_set, robot_config, distance_map);

		closed_set.push_back(current_state);

		}

	}
	//Limpar a pilha do closed_set.
    while(!closed_set.empty())
    {
    	state_node *tmp = closed_set.back();
        closed_set.pop_back();
        delete tmp;
    }

	printf("Terminou compute_astar_path !\n");
//	exit(1);

}


static void
carmen_rddf_play_get_parameters(int argc, char** argv)
{
	carmen_param_t param_list[] =
	{
			{(char *) "robot",	(char *) "length",								  		CARMEN_PARAM_DOUBLE, &robot_config.length,							 			1, NULL},
			{(char *) "robot",	(char *) "width",								  		CARMEN_PARAM_DOUBLE, &robot_config.width,								 			1, NULL},
			{(char *) "robot", 	(char *) "distance_between_rear_wheels",		  		CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_wheels,			 		1, NULL},
			{(char *) "robot", 	(char *) "distance_between_front_and_rear_axles", 		CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 		1, NULL},
			{(char *) "robot", 	(char *) "distance_between_front_car_and_front_wheels",	CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_car_and_front_wheels,	1, NULL},
			{(char *) "robot", 	(char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_car_and_rear_wheels,		1, NULL},
			{(char *) "robot", 	(char *) "max_velocity",						  		CARMEN_PARAM_DOUBLE, &robot_config.max_v,									 		1, NULL},
			{(char *) "robot", 	(char *) "max_steering_angle",					  		CARMEN_PARAM_DOUBLE, &robot_config.max_phi,								 		1, NULL},
			{(char *) "robot", 	(char *) "maximum_acceleration_forward",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_acceleration_reverse",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_reverse,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_deceleration_forward",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_forward,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_deceleration_reverse",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_reverse,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_steering_command_rate",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_steering_command_rate,					1, NULL},
			{(char *) "robot", 	(char *) "understeer_coeficient",						CARMEN_PARAM_DOUBLE, &robot_config.understeer_coeficient,							1, NULL}
		};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	carmen_rddf_play_get_parameters(argc, argv);

	carmen_rddf_play_subscribe_messages();

	signal(SIGINT, shutdown_module);

	printf("Até aqui está funcionando!\n");

	carmen_ipc_dispatch();

	return (0);
}

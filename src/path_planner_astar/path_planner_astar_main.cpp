#include "path_planner_astar.h"

#define THETA_SIZE 1
#define ASTAR_GRID_RESOLUTION 1.0

#define ACKERMAN_EXPANSION 0

#define MAX_VIRTUAL_LASER_SAMPLES 100000
carmen_mapper_virtual_laser_message virtual_laser_message;

carmen_point_t *final_goal = NULL;
carmen_localize_ackerman_globalpos_message *current_globalpos_msg = NULL;
carmen_robot_ackerman_config_t robot_config;
carmen_obstacle_distance_mapper_map_message distance_map;
state_node_p ***astar_map;

using namespace std;
using namespace cv;
using namespace boost::heap;

Mat map_image;



void
draw_astar_object(carmen_ackerman_traj_point_t *object_pose, int color)
{
	/*

	  CARMEN_RED = 0,
	  CARMEN_BLUE = 1,
	  CARMEN_WHITE = 2,
	  CARMEN_YELLOW = 3,
	  CARMEN_GREEN = 4,
	  CARMEN_LIGHT_BLUE = 5,
	  CARMEN_BLACK = 6,
	  CARMEN_ORANGE = 7,
	  CARMEN_GREY = 8,
	  CARMEN_LIGHT_GREY = 9,
	  CARMEN_PURPLE = 10,

	 */
	virtual_laser_message.positions[virtual_laser_message.num_positions].x = object_pose->x;
	virtual_laser_message.positions[virtual_laser_message.num_positions].y = object_pose->y;
	virtual_laser_message.colors[virtual_laser_message.num_positions] = color;
	virtual_laser_message.num_positions++;
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

    return(state_path);
}



std::vector<carmen_ackerman_traj_point_t>
build_rddf_poses(std::vector<state_node> &path, state_node *current_state, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	path = build_state_path(current_state->parent);
	std::reverse(path.begin(), path.end());
	std::vector<carmen_ackerman_traj_point_t> temp_rddf_poses_from_path;

	for (int i = 0; i < path.size(); i++)
	{
		temp_rddf_poses_from_path.push_back(path[i].state);
		draw_astar_object(&path[i].state, 4);

	}

	return temp_rddf_poses_from_path;
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
astar_publish_rddf_message(state_node *current_state,  carmen_obstacle_distance_mapper_map_message *distance_map)
{
	std::vector<state_node> path;
	std::vector<carmen_ackerman_traj_point_t> temp_rddf_poses_from_path = build_rddf_poses(path, current_state, distance_map);
	carmen_ackerman_traj_point_t *carmen_rddf_poses_from_path = &temp_rddf_poses_from_path[0];
	carmen_ackerman_traj_point_t last_pose = {.x = 0.0, .y = 0.0, .theta = 0.0, .v = 9.0, .phi=0.2};
	int annotations[2] = {1, 2};
	int annotation_codes[2] = {1, 2};
/*
	for (int i = 0; i < path.size(); i++)
	{
		printf("Poses do rddf: %f %f %d\n", carmen_rddf_poses_from_path[i].x, carmen_rddf_poses_from_path[i].y, i);
	//	printf("Poses do path: %f %f %d\n", path[i].state.x, path[i].state.y, i);
	}
*/
	carmen_rddf_publish_road_profile_message(carmen_rddf_poses_from_path, &last_pose, path.size(), 1, annotations, annotation_codes);
	printf("Poses enviadas!\n");
	temp_rddf_poses_from_path.clear();
}


void
publish_astar_draw()
{
	virtual_laser_message.host = carmen_get_host();
	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());
//	virtual_laser_message.num_positions = 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////


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
	new_state->is_open = 1;
	new_state->was_visited = 1;
	new_state->is_obstacle = 0;

	return (new_state);
}


static void
get_pos_message(carmen_point_t robot_pose)
{
	if (final_goal != NULL)
	{
		printf("Pos_message_handler: %lf %lf %lf\n", robot_pose.x, robot_pose.y, robot_pose.theta);
		compute_astar_path(&robot_pose, final_goal, robot_config , &distance_map);
		final_goal = NULL;
	}
}


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


int
get_distance_map_x(int x, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	return round((double) (x * distance_map->config.resolution) + distance_map->config.x_origin);
}


int
get_distance_map_y(int y, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	return round((double) (y * distance_map->config.resolution) + distance_map->config.y_origin);
}


void
alloc_astar_map(carmen_obstacle_distance_mapper_map_message *distance_map)
{
	int i, j, z;
	int theta_size = THETA_SIZE;
	int x_size = round(distance_map->config.x_size / ASTAR_GRID_RESOLUTION + 1);
	int y_size = round(distance_map->config.y_size / ASTAR_GRID_RESOLUTION + 1);
	double pos_x = 0.0;
	double pos_y = 0.0;
	printf("sizemap = %d %d \n", x_size, y_size);
	astar_map = (state_node_p ***)calloc(x_size, sizeof(state_node_p**));
	carmen_test_alloc(astar_map);

	for (i = 0; i < x_size; i++)
	{
		astar_map[i] = (state_node_p **)calloc(y_size, sizeof(state_node_p*));
		carmen_test_alloc(astar_map[i]);

		for (j = 0; j < y_size; j++)
		{
			astar_map[i][j] = (state_node_p*)calloc(theta_size, sizeof(state_node_p));
			carmen_test_alloc(astar_map[i][j]);

			for (z = 0; z < THETA_SIZE; z++)
			{
				astar_map[i][j][z]= (state_node_p) malloc(sizeof(state_node));
				carmen_test_alloc(astar_map[i][j][z]);
				pos_x = get_distance_map_x(i, distance_map);
				pos_y = get_distance_map_y(j, distance_map);
				if(obstacle_distance(pos_x, pos_y, distance_map) < 2.0)
					astar_map[i][j][z]->is_obstacle = 1;
				else
					astar_map[i][j][z]->is_obstacle = 0;

				astar_map[i][j][z]->was_visited = 0;

			}
		}
	}
}

void
clear_astar_map(carmen_obstacle_distance_mapper_map_message *distance_map)
{
	int i, j, k, z;
	int x_size = round(distance_map->config.x_size / ASTAR_GRID_RESOLUTION + 1);
	int y_size = round(distance_map->config.y_size / ASTAR_GRID_RESOLUTION + 1);

	for (i = 0; i < x_size; i++)
		for (j = 0; j < y_size; j++)
			for (k = 0; k < THETA_SIZE; k++)
				astar_map[i][j][k] = NULL;
}


int
get_astar_map_theta(double theta)
{
	theta = theta < 0 ? (2 * M_PI + theta) : theta;
	int resolution = (int) round(360/THETA_SIZE);

	return  (int)round((carmen_radians_to_degrees(theta) / resolution)) % (int)round(360 / resolution);
}


int
get_astar_map_x(double x, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	return round((double) (x - distance_map->config.x_origin) / distance_map->config.resolution);
}


int
get_astar_map_y(double y, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	return round((double) (y - distance_map->config.y_origin) / distance_map->config.resolution);
}


bool
my_list_ordenation (state_node *a, state_node *b)
{
	return (a->g+ a->h > b->g + b->h);
}


discrete_pos_node*
get_current_pos(state_node* current_state, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	discrete_pos_node *current_pos = (discrete_pos_node*) malloc(sizeof(discrete_pos_node));
	current_pos->x = get_astar_map_x(current_state->state.x, distance_map);
	current_pos->y = get_astar_map_y(current_state->state.y, distance_map);
	current_pos->theta = get_astar_map_theta(current_state->state.theta);

	return current_pos;
}


void
insert_open(vector<state_node*> &open, state_node *current_state, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	open.push_back(current_state);
	discrete_pos_node *current_pos = get_current_pos(current_state, distance_map);
	astar_map[current_pos->x][current_pos->y][current_pos->theta] = current_state;
}


state_node*
get_lowest_rank(vector<state_node*> &open)
{
	state_node *current_state = open.back();
	return current_state;
}


int
is_goal(state_node* current_state, state_node* goal_state)
{
	if(DIST2D(current_state->state, goal_state->state) < 1.5)
		return 1;
	else
		return 0;
}


state_node*
pop_lowest_rank(vector<state_node*> &open)
{
	state_node *current_state = open.back();
	open.pop_back();
	return current_state;
}


vector<state_node*>
expansion(state_node *current, carmen_obstacle_distance_mapper_map_message *distance_map)
{
    double add_x[3] = {-1.0, 0.0, 1.0};
    double add_y[3] = {-1.0, 0.0, 1.0};
    vector<state_node*> neighbor;

    double target_phi, distance_traveled = 0.0;
    distance_traveled = 2.0;
    double steering_acceleration[3] = {-0.25, 0.0, 0.25}; //TODO ler velocidade angular do volante do carmen.ini
    double target_v[2]   = {2.0, -2.0};
    int size_for;

    if (ACKERMAN_EXPANSION)
        	size_for = sizeof(target_v)/sizeof(target_v[0]);
        else
        	size_for = 3;

    for (int i = 0; i<size_for; i++)
    {
    	for (int j = 0; j<3; j++)
    	{
        	state_node_p new_state = (state_node_p) malloc(sizeof(state_node));
        	if(ACKERMAN_EXPANSION)
        	{
        		target_phi = carmen_clamp(-robot_config.max_phi, (current->state.phi + steering_acceleration[j]), robot_config.max_phi);
        		new_state->state = carmen_libcarmodel_recalc_pos_ackerman(current->state, target_v[i], target_phi, 1.0 , &distance_traveled, DELTA_T, robot_config);
        	}
        	else
        	{
        		new_state->state.x = current->state.x + add_x[i];
				new_state->state.y = current->state.y + add_y[j];
				new_state->state.theta = current->state.theta;
        	}
			discrete_pos_node *current_pos = get_current_pos(new_state, distance_map);
			if(astar_map[current_pos->x][current_pos->y][current_pos->theta]->is_obstacle == 1)
			{
				free(new_state);
			}
			else
			{
				draw_astar_object(&new_state->state, 0);
				neighbor.push_back(new_state);
			}
    	}
    }
    publish_astar_draw();
    return neighbor;
}


double
g(state_node *current)
{
	return current->g;
}


double
h(state_node *current, state_node *goal)
{
	return DIST2D(current->state, goal->state);
}


double
movementcost(state_node *current, state_node *neighbor)
{
	return DIST2D(current->state, neighbor->state);
}

int
node_exist(vector<state_node*> &list, state_node *current, carmen_obstacle_distance_mapper_map_message *distance_map)
{
	for (int i = 0; i < list.size(); i++)
	{
		discrete_pos_node *current_pos = get_current_pos(current, distance_map);
		discrete_pos_node *list_pos = get_current_pos(list[i], distance_map);
		if(current_pos->x == list_pos->x && current_pos->y == list_pos->y)
		{
			return i+1;
		}
	}
	return 0;
}

void
compute_astar_path(carmen_point_t *robot_pose, carmen_point_t *goal_pose, carmen_robot_ackerman_config_t robot_config,
		carmen_obstacle_distance_mapper_map_message *distance_map)
{
	int it_number = 0;
	int indice = 0;
	double cost = 0.0;
	vector<state_node*> neighbor;
	state_node *start_state, *goal_state, *current;
	start_state = create_state_node(robot_pose->x, robot_pose->y, robot_pose->theta, 0.1, 0.0, 0.0, DIST2D_P(robot_pose, goal_pose), NULL);
	goal_state = create_state_node(goal_pose->x, goal_pose->y, goal_pose->theta, 0.0, 0.0, DBL_MAX, DBL_MAX, NULL);
	alloc_astar_map(distance_map);
	vector<state_node*> open;

	// Following the code from: http://theory.stanford.edu/~amitp/GameProgramming/ImplementationNotes.html
	insert_open(open, start_state, distance_map);
	vector<state_node*> closed;
	while (!open.empty() && is_goal(get_lowest_rank(open), goal_state) != 1)
	{
		current = pop_lowest_rank(open);
		closed.push_back(current);
		neighbor = expansion(current, distance_map);
		while(it_number< neighbor.size())
		{
			cost = g(current) + movementcost(current, neighbor[it_number]);
			indice = node_exist(open, neighbor[it_number], distance_map);
			if(indice != 0 && cost < g(neighbor[it_number]))
			{
				open.erase(open.begin() + (indice-1));
			}
			indice = node_exist(closed, neighbor[it_number], distance_map);
			if(indice != 0 && cost < g(neighbor[it_number]))
			{
				closed.erase(closed.begin() + (indice-1));
			}

			if(node_exist(open, neighbor[it_number], distance_map) == 0 && node_exist(closed, neighbor[it_number], distance_map) == 0 )
			{
				neighbor[it_number]->g = cost;
				neighbor[it_number]->h = h(neighbor[it_number], goal_state);
				neighbor[it_number]->parent = current;
				open.push_back(neighbor[it_number]);
				sort(open.begin(), open.end(), my_list_ordenation);
			}
			it_number++;
		}
		it_number = 0;
		neighbor.clear();
	}

	if(open.empty())
		printf("Caminho não encontrado\n");
	else
	{
		astar_publish_rddf_message(current, distance_map);
		publish_astar_draw();
	}

	open.clear();
	closed.clear();
	clear_astar_map(distance_map);
	virtual_laser_message.num_positions = 0;
	printf("Terminou compute_astar_path !\n");
//	exit(1);
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	current_globalpos_msg = msg;
	get_pos_message(msg->globalpos/*, msg->v, msg->timestamp*/);
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
	static carmen_obstacle_distance_mapper_compact_map_message *compact_distance_map = NULL;

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
}


///////////////////////////////////////////////////////////////////////////////////////////////


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


void
path_planner_astar_initialize()
{
	memset(&virtual_laser_message, 0, sizeof(carmen_mapper_virtual_laser_message));
	virtual_laser_message.positions = (carmen_position_t *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(carmen_position_t));
	virtual_laser_message.colors = (char *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(char));
	virtual_laser_message.host = carmen_get_host();
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

	path_planner_astar_initialize();

	signal(SIGINT, shutdown_module);

	printf("Até aqui está funcionando!\n");

	carmen_ipc_dispatch();

	return (0);
}

/*
 * model_predictive_planner_main.cpp
 *
 *  Created on: 04/12/2012
 *      Author: romulo
 */

#include <carmen/carmen.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/grid_mapping_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/ford_escape_hybrid_interface.h>

#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/grid_mapping.h>
#include <prob_map.h>

#include "model/robot_config.h"
#include "model/global_state.h"
#include "model/rrt_node.h"
#include "model/pose.h"

#include "util.h"
#include "publisher_util.h"
#include "model_predictive_planner.h"

#define DIST_SQR(x1,y1,x2,y2) ((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

carmen_rddf_annotation_message last_rddf_annotation_message;
Tree tree; //tree rooted on robot
TrajectoryLookupTable *g_trajectory_lookup_table;
carmen_rddf_road_profile_message goal_list_message;

static int update_lookup_table = 0;


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_model_predictive_rrt_path_message(list<RRT_Path_Edge> path)
{
	int i = 0;
	rrt_path_message msg;
	list<RRT_Path_Edge>::iterator it;

	msg.host  = carmen_get_host();
	msg.timestamp = GlobalState::localizer_pose_timestamp;
	msg.last_goal = GlobalState::last_goal ? 1 : 0;

	if (GlobalState::goal_pose)
	{
		msg.goal.x = GlobalState::goal_pose->x;
		msg.goal.y = GlobalState::goal_pose->y;
		msg.goal.theta = GlobalState::goal_pose->theta;
	}
	else
	{
		msg.goal.x = msg.goal.y = msg.goal.theta = 0.0;
	}

	if (path.empty())
	{
		// return;
		msg.size = 0;
		msg.path = NULL;
	}
	else
	{
		msg.size = path.size();
		msg.path = (Edge_Struct *) malloc(sizeof(Edge_Struct) * msg.size);
	}

	for (it = path.begin(); it != path.end(); it++, i++)
	{
		msg.path[i].p1.x = it->p1.pose.x;
		msg.path[i].p1.y = it->p1.pose.y;
		msg.path[i].p1.theta = it->p1.pose.theta;
		msg.path[i].p1.v = it->p1.v_and_phi.v;
		msg.path[i].p1.phi = it->p1.v_and_phi.phi;

		msg.path[i].p2.x = it->p2.pose.x;
		msg.path[i].p2.y = it->p2.pose.y;
		msg.path[i].p2.theta = it->p2.pose.theta;
		msg.path[i].p2.v = it->p2.v_and_phi.v;
		msg.path[i].p2.phi = it->p2.v_and_phi.phi;

		msg.path[i].v = it->command.v;
		msg.path[i].phi = it->command.phi;
		msg.path[i].time = it->time;
	}

	Publisher_Util::publish_rrt_path_message(&msg);

	free(msg.path);
}


void
publish_model_predictive_planner_motion_commands(vector<carmen_ackerman_path_point_t> path)
{
	if (!GlobalState::following_path)
		return;

	carmen_ackerman_motion_command_t* commands =
			(carmen_ackerman_motion_command_t*) (malloc(path.size() * sizeof(carmen_ackerman_motion_command_t)));
	int i = 0;
	for (std::vector<carmen_ackerman_path_point_t>::iterator it = path.begin();	it != path.end(); ++it)
	{
		commands[i].v = it->v;
		commands[i].phi = it->phi;
		commands[i].time = it->time;

		i++;
	}

	int num_commands = path.size();
	if (GlobalState::use_obstacle_avoider)
		carmen_robot_ackerman_publish_motion_command(commands, num_commands);
	else
		carmen_base_ackerman_publish_motion_command(commands, num_commands);

	free(commands);
}


void
publish_path_follower_motion_commands(carmen_ackerman_motion_command_t *commands, int num_commands)
{
//	system("clear");
//	for (int i = 0; (i < num_commands) && (i < 20); i++)
//		printf("v = %2.2lf, phi = %2.2lf, t = %2.3lf\n", commands[i].v, carmen_radians_to_degrees(commands[i].phi), commands[i].time);
//	fflush(stdout);

	if (GlobalState::use_obstacle_avoider)
		carmen_robot_ackerman_publish_motion_command(commands, num_commands);
	else
		carmen_base_ackerman_publish_motion_command(commands, num_commands);
}


void
publish_path_follower_single_motion_command(double v, double phi)
{
	carmen_ackerman_motion_command_t commands[2];

	commands[0].v = v;
	commands[0].phi = phi;
	commands[0].time = 0.5;
	commands[1] = commands[0];
	publish_path_follower_motion_commands(commands, 2);
}


void
publish_model_predictive_planner_single_motion_command(double v, double phi)
{
	vector<carmen_ackerman_path_point_t> path;

	carmen_ackerman_path_point_t traj;
	traj.v = v;
	traj.phi = phi;
	traj.time = 1.0;
	path.push_back(traj);
	path.push_back(traj);
	publish_model_predictive_planner_motion_commands(path);

	publish_path_follower_single_motion_command(0.0, GlobalState::last_odometry.phi);
}


void
publish_navigator_ackerman_plan_message(carmen_ackerman_traj_point_t *path, int path_size)
{
	carmen_navigator_ackerman_plan_message msg;

	msg.host = carmen_get_host();
	msg.timestamp = GlobalState::localizer_pose_timestamp;
	msg.path_length = path_size;
	msg.path = path;

	Publisher_Util::publish_navigator_ackerman_plan_message(msg);
}


void
publish_navigator_ackerman_status_message()
{
	if (!GlobalState::localizer_pose)
	{
		return;
	}

	IPC_RETURN_TYPE err = IPC_OK;

	static bool first_time = true;

	if (first_time)
	{
		err = IPC_defineMsg(
				(char *)CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME,
				IPC_VARIABLE_LENGTH,
				(char *)CARMEN_NAVIGATOR_ACKERMAN_STATUS_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME);
	}

	carmen_navigator_ackerman_status_message msg;
	msg.autonomous = GlobalState::following_path;
	msg.goal_set   = GlobalState::goal_pose != NULL;

	if (msg.goal_set)
	{
		msg.goal.x = GlobalState::goal_pose->x;
		msg.goal.y = GlobalState::goal_pose->y;
		msg.goal.theta = GlobalState::goal_pose->theta;
	}
	else
	{
		msg.goal.theta = 0;
		msg.goal.y	   = 0;
		msg.goal.x	   = 0;
	}

	msg.host		= carmen_get_host();
	msg.robot.x		= GlobalState::localizer_pose->x;
	msg.robot.y		= GlobalState::localizer_pose->y;
	msg.robot.theta = GlobalState::localizer_pose->theta;
	msg.robot.v		= GlobalState::last_odometry.v;
	msg.robot.phi	= GlobalState::last_odometry.phi;
	msg.timestamp	= GlobalState::localizer_pose_timestamp;

	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME, &msg);

	carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME);
}


void
publish_plan_tree_for_navigator_gui(Tree tree)
{
	if (GlobalState::current_algorithm == CARMEN_BEHAVIOR_SELECTOR_RRT)
		if (GlobalState::publish_tree)
			Publisher_Util::publish_plan_tree_message(tree);
}

///////////////////////////////////////////////////////////////////////////////////////////////


void
free_tree(Tree *tree)
{
	if (tree->paths != NULL)
	{
		for (int i = 0; i < tree->num_paths; i++)
			free(tree->paths[i]);
		free(tree->paths);
		free(tree->paths_sizes);
	}

	tree->num_edges = 0;
}


void
copy_path_to_traj(carmen_ackerman_traj_point_t *traj, vector<carmen_ackerman_path_point_t> path)
{
	int i = 0;
	for (std::vector<carmen_ackerman_path_point_t>::iterator it = path.begin(); it != path.end(); ++it, ++i)
	{
		traj[i].x = it->x;
		traj[i].y = it->y;
		traj[i].theta = it->theta;
		traj[i].v = it->v;
		traj[i].phi = it->phi;
	}
}


vector<carmen_ackerman_path_point_t>
compute_plan(Tree *tree)
{
	if (goal_list_message.number_of_poses == 0)
	{
		printf("Error: trying to compute plan without rddf\n");
		return vector<carmen_ackerman_path_point_t>();
	}

	free_tree(tree);
	vector<vector<carmen_ackerman_path_point_t>> path = ModelPredictive::compute_path_to_goal(GlobalState::localizer_pose,
			GlobalState::goal_pose, GlobalState::last_odometry, GlobalState::robot_config.max_v, &goal_list_message);

	if (path.size() == 0)
	{
		tree->num_paths = 0;
		tree->paths = NULL;
		vector<carmen_ackerman_path_point_t> voidVector;
		return(voidVector);
	}

	tree->num_paths = path.size();
	tree->paths = (carmen_ackerman_traj_point_t **) malloc(tree->num_paths * sizeof(carmen_ackerman_traj_point_t *));
	tree->paths_sizes = (int *) malloc(tree->num_paths * sizeof(int));

	for (unsigned int i = 0; i < path.size(); i++)
	{
		tree->paths[i] = (carmen_ackerman_traj_point_t *) malloc(path[i].size() * sizeof(carmen_ackerman_traj_point_t));
		copy_path_to_traj(tree->paths[i], path[i]);
		tree->paths_sizes[i] = path[i].size();
	}

	if (!GlobalState::last_plan_pose)
		GlobalState::last_plan_pose = new Pose();
	*GlobalState::last_plan_pose = *GlobalState::localizer_pose;

	return (path[0]);
}


void
go()
{
	GlobalState::following_path = true;
}


void
stop()
{
	GlobalState::following_path = false;
	publish_model_predictive_planner_single_motion_command(0.0, 0.0);
}


void
compute_obstacles_rtree(carmen_map_server_compact_cost_map_message *map)
{

//	static double p_x_o = 0.0;
//	static double p_y_o = 0.0;

	if (GlobalState::localizer_pose && GlobalState::goal_pose)// &&
//		p_x_o != GlobalState::cost_map.config.x_origin &&
//		p_y_o != GlobalState::cost_map.config.y_origin)
	{
		GlobalState::obstacles_rtree.clear();

		int px = (GlobalState::localizer_pose->x - GlobalState::cost_map.config.x_origin) / GlobalState::cost_map.config.resolution;
		int py = (GlobalState::localizer_pose->y - GlobalState::cost_map.config.y_origin) / GlobalState::cost_map.config.resolution;
		int gx = (GlobalState::goal_pose->x - GlobalState::cost_map.config.x_origin) / GlobalState::cost_map.config.resolution;
		int gy = (GlobalState::goal_pose->y - GlobalState::cost_map.config.y_origin) / GlobalState::cost_map.config.resolution;
		int margin = 0.0 / GlobalState::cost_map.config.resolution;
		int sqr_d = DIST_SQR(px,py,gx,gy) + margin * margin;
		int count = 0;
		int total = 0;
		for (int i = 0; i < map->size; i += 1)
		{
			if (map->value[i] > 0.5)
			{
				if ((DIST_SQR(px,py,map->coord_x[i],map->coord_y[i]) < sqr_d) &&
					(DIST_SQR(gx,gy,map->coord_x[i],map->coord_y[i]) < sqr_d))
				{
					occupied_cell map_cell = occupied_cell(
							(double) map->coord_x[i] * GlobalState::cost_map.config.resolution,
							(double) map->coord_y[i] * GlobalState::cost_map.config.resolution);
					GlobalState::obstacles_rtree.insert(map_cell);
					count++;
				}
				total++;
			}
		}
//		p_x_o = GlobalState::cost_map.config.x_origin;
//		p_y_o = GlobalState::cost_map.config.y_origin;
//		printf("fraction = %lf\n", (double) count / (double) total);
		fflush(stdout);
	}
}

void
compute_obstacles_kdtree(carmen_map_server_compact_cost_map_message *map)
{

	//	static double p_x_o = 0.0;
	//	static double p_y_o = 0.0;

	std::vector<Point2D> obstacles;
	Point2D point;

	if (GlobalState::localizer_pose && GlobalState::goal_pose)// &&
		//		p_x_o != GlobalState::cost_map.config.x_origin &&
		//		p_y_o != GlobalState::cost_map.config.y_origin)
	{
		int px = (GlobalState::localizer_pose->x - GlobalState::cost_map.config.x_origin) / GlobalState::cost_map.config.resolution;
		int py = (GlobalState::localizer_pose->y - GlobalState::cost_map.config.y_origin) / GlobalState::cost_map.config.resolution;
		int gx = (GlobalState::goal_pose->x - GlobalState::cost_map.config.x_origin) / GlobalState::cost_map.config.resolution;
		int gy = (GlobalState::goal_pose->y - GlobalState::cost_map.config.y_origin) / GlobalState::cost_map.config.resolution;
		int margin = 3.0 / GlobalState::cost_map.config.resolution;
		int sqr_d = DIST_SQR(px,py,gx,gy) + margin * margin;
		int count = 0;
		int total = 0;
		for (int i = 0; i < map->size; i += 1)
		{
			if (map->value[i] > 0.5)
			{
				if ((DIST_SQR(px,py,map->coord_x[i],map->coord_y[i]) < sqr_d) &&
						(DIST_SQR(gx,gy,map->coord_x[i],map->coord_y[i]) < sqr_d))
				{
					point.position[0] = (double) map->coord_x[i] * GlobalState::cost_map.config.resolution;
					point.position[1] = (double) map->coord_y[i] * GlobalState::cost_map.config.resolution;

					obstacles.push_back(point);
					count++;
				}
				total++;
			}

		}
			// insert the obstacles points, quick-select approach
		GlobalState::obstacles_kdtree.rebuild(obstacles);
	}
}

list<RRT_Path_Edge>
build_path_follower_path(vector<carmen_ackerman_path_point_t> path)
{
	list<RRT_Path_Edge> path_follower_path;
	RRT_Path_Edge path_edge;

	if (path.size() < 2)
		return (path_follower_path);

	for (unsigned int i = 0; i < path.size() - 1; i++)
	{
		path_edge.p1.pose.x = path[i].x;
		path_edge.p1.pose.y = path[i].y;
		path_edge.p1.pose.theta = path[i].theta;
		path_edge.p1.v_and_phi.v = path[i].v;
		path_edge.p1.v_and_phi.phi = path[i].phi;

		path_edge.p2.pose.x = path[i + 1].x;
		path_edge.p2.pose.y = path[i + 1].y;
		path_edge.p2.pose.theta = path[i + 1].theta;
		path_edge.p1.v_and_phi.v = path[i + 1].v;
		path_edge.p1.v_and_phi.phi = path[i + 1].phi;

		path_edge.command.v = path[i + 1].v;
		path_edge.command.phi = path[i + 1].phi;
		path_edge.time = path[i].time;

		path_follower_path.push_back(path_edge);
	}

	return (path_follower_path);
}


void
build_and_follow_path()
{
	list<RRT_Path_Edge> path_follower_path;

	if (GlobalState::goal_pose && (GlobalState::current_algorithm == CARMEN_BEHAVIOR_SELECTOR_RRT))
	{
		vector<carmen_ackerman_path_point_t> path = compute_plan(&tree);
		if (tree.num_paths > 0 && path.size() > 0)
		{
			path_follower_path = build_path_follower_path(path);
			publish_model_predictive_rrt_path_message(path_follower_path);
			publish_navigator_ackerman_plan_message(tree.paths[0], tree.paths_sizes[0]);
		}
//		else
//			publish_path_follower_single_motion_command(0.0, GlobalState::last_odometry.phi);
//		{
//			if (GlobalState::last_odometry.v == 0.0)
//				publish_path_follower_single_motion_command(0.0, 0.0);
//			else
//				publish_path_follower_single_motion_command(0.0, GlobalState::last_odometry.phi);
//		}
		publish_plan_tree_for_navigator_gui(tree);
		publish_navigator_ackerman_status_message();
	}
//	else
//	{
//		if (!GlobalState::goal_pose)
//			printf("NO GOAL!!!\n");
//	}
}


void
build_and_follow_path_old()
{
	if (GlobalState::goal_pose && (GlobalState::current_algorithm == CARMEN_BEHAVIOR_SELECTOR_RRT))
	{
		vector<carmen_ackerman_path_point_t> path = compute_plan(&tree);
		if (tree.num_paths > 0 && path.size() > 0)
		{
			publish_model_predictive_planner_motion_commands(path);
			publish_navigator_ackerman_plan_message(tree.paths[0], tree.paths_sizes[0]);
		}
		else
			publish_path_follower_single_motion_command(0.0, GlobalState::last_odometry.phi);

		publish_plan_tree_for_navigator_gui(tree);
		publish_navigator_ackerman_status_message();
	}
}


void
create_map_obstacle_mask()
{
	//limites de x = comprimento/celula e y = lagura / tamanho_celula
	// rotacao xnew = R*cos
	//  newPoint.x = Math.cos(convertDegreesToRadians(angle))* lenght;
	//newPoint.y = Math.sin(convertDegreesToRadians(angle))* lenght;

	int topLimit = ceil(((GlobalState::robot_config.distance_between_rear_wheels) / 2) / GlobalState::cost_map.config.resolution);
	int bottomLimit = -1 * topLimit;
	int rigthLimit = ceil((GlobalState::robot_config.distance_between_front_and_rear_axles + GlobalState::robot_config.distance_between_front_car_and_front_wheels) / GlobalState::cost_map.config.resolution);
	int leftLimit = -1 * GlobalState::robot_config.distance_between_rear_car_and_rear_wheels / GlobalState::cost_map.config.resolution;

	int angle_max = 1;
	for (int angulo = 0 ; angulo < angle_max; angulo++)
	{
		vector<cell_coords_t> points;
//		max_x = calculo do angulo;
//		max_y = calculo do angulo;
	    for (int j = bottomLimit; j <= topLimit; j++)
	    {
	        for (int i = leftLimit; i <= rigthLimit; i++)
	        {
	        	cell_coords_t point;
	        	point.x = (i * cos(angulo)) - (sin(angulo) * j);
	        	point.y = j;
	        	points.push_back(point);
//	        	printf("%d,%d ", i,j);
			}
//	    	printf("\n");
		}

	    GlobalState::cell_mask.push_back(points);
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	//printf("tempo da localizacao: %lf\n", msg->timestamp);

	Pose pose = Util::convert_to_pose(msg->globalpos);
	GlobalState::set_robot_pose(pose, msg->timestamp);

	build_and_follow_path();
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
//	printf("tempo da localizacao: %lf\n", msg->timestamp);

	Pose pose = Util::convert_to_pose(msg->truepose);
	GlobalState::set_robot_pose(pose, msg->timestamp);

	build_and_follow_path();
}


static void
navigator_ackerman_set_goal_message_handler(carmen_navigator_ackerman_set_goal_message *msg)
{
	//na mensagem atual não é possível representar um goal nulo
	if (msg->x == -1 && msg->y == -1 && msg->theta == 0)
	{
		GlobalState::goal_pose = NULL;
		return;
	}

	Pose goal_pose;
	goal_pose.x		= msg->x;
	goal_pose.y		= msg->y;
	goal_pose.theta = carmen_normalize_theta(msg->theta);
	GlobalState::set_goal_pose(goal_pose);

	GlobalState::last_goal = true;
}


static void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *msg)
{
	GlobalState::last_odometry.v = msg->v;
	GlobalState::last_odometry.phi = msg->phi;
}


static void
behaviour_selector_goal_list_message_handler(carmen_behavior_selector_goal_list_message *msg)
{
	Pose goal_pose;

	if ((msg->size <= 0) || !msg->goal_list || !GlobalState::localizer_pose)
	{
		printf("Empty goal list or localize not received\n");
		return;
	}

	GlobalState::last_goal = (msg->size == 1)? true: false;

	goal_pose.x = msg->goal_list->x;
	goal_pose.y = msg->goal_list->y;
	goal_pose.theta = carmen_normalize_theta(msg->goal_list->theta);

	// Map annotations handling
	double distance_to_annotation = DIST2D(last_rddf_annotation_message.annotation_point, *GlobalState::localizer_pose);
	if (((last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_BUMP) ||
		 (last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_BARRIER)) &&
		(distance_to_annotation < 30.0))
		GlobalState::robot_config.max_v = 1.0;
	else
		GlobalState::robot_config.max_v = fmin(msg->goal_list->v, GlobalState::param_max_vel);

//	printf("vgoal = %lf\n", GlobalState::robot_config.max_vel);

	GlobalState::set_goal_pose(goal_pose);
}


static void
behavior_selector_state_message_handler(carmen_behavior_selector_state_message *msg)
{
	GlobalState::behavior_selector_state = msg->state;
	GlobalState::current_algorithm = msg->algorithm;
}


static void
map_server_compact_cost_map_message_handler(carmen_map_server_compact_cost_map_message *message)
{
	static carmen_compact_map_t *compact_cost_map = NULL;

	if (compact_cost_map == NULL)
	{
		carmen_grid_mapping_create_new_map(&GlobalState::cost_map, message->config.x_size, message->config.y_size, message->config.resolution);
		memset(GlobalState::cost_map.complete_map, 0, GlobalState::cost_map.config.x_size * GlobalState::cost_map.config.y_size * sizeof(double));

		compact_cost_map = (carmen_compact_map_t *) (calloc(1, sizeof(carmen_compact_map_t)));
		carmen_cpy_compact_cost_message_to_compact_map(compact_cost_map, message);
		carmen_prob_models_uncompress_compact_map(&GlobalState::cost_map, compact_cost_map);
	}
	else
	{
		carmen_prob_models_clear_carmen_map_using_compact_map(&GlobalState::cost_map, compact_cost_map, 0.0);
		carmen_prob_models_free_compact_map(compact_cost_map);
		carmen_cpy_compact_cost_message_to_compact_map(compact_cost_map, message);
		carmen_prob_models_uncompress_compact_map(&GlobalState::cost_map, compact_cost_map);
	}

	GlobalState::cost_map.config = message->config;

//	compute_obstacles_rtree(message);
//	compute_obstacles_kdtree(message);

//	if(!GlobalState::cost_map_initialized)
//		create_map_obstacle_mask();
	GlobalState::cost_map_initialized = true;
}


static void
carmen_grid_mapping_distance_map_message_handler(carmen_grid_mapping_distance_map_message *message)
{
	GlobalState::distance_map = message;
}


void
ford_escape_status_handler(carmen_ford_escape_status_message *msg)
{
	//TODO tratar tambem comandos de ultrapassagem (ou talvez tratar no behavior selector)
	GlobalState::ford_escape_status.g_XGV_turn_signal = msg->g_XGV_turn_signal;
	//Tratando se o navegador esta em no modo real ou em modo simulacao
	GlobalState::ford_escape_online = true;
}


void
rddf_message_handler(/*carmen_rddf_road_profile_message *message*/)
{
//	printf("RDDF NUM POSES: %d \n", message->number_of_poses);
//
//	for (int i = 0; i < message->number_of_poses; i++)
//	{
//		printf("RDDF %d: x  = %lf, y = %lf , theta = %lf\n", i, message->poses[i].x, message->poses[i].y, message->poses[i].theta);
//		//getchar();
//	}
}


static void
navigator_ackerman_go_message_handler()
{
	go();
}


static void
navigator_ackerman_stop_message_handler()
{
	stop();
}


static void
signal_handler(int sig)
{
	printf("Signal %d received, exiting program ...\n", sig);
	if (update_lookup_table)
	{
		save_trajectory_lookup_table();
		printf("New trajectory_lookup_table.bin saved.\n");
	}

	exit(1);
}
///////////////////////////////////////////////////////////////////////////////////////////////


void
register_handlers_specific()
{
	carmen_subscribe_message(
			(char *)CARMEN_NAVIGATOR_ACKERMAN_GO_NAME,
			(char *)CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_navigator_ackerman_go_message),
			(carmen_handler_t)navigator_ackerman_go_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *)CARMEN_NAVIGATOR_ACKERMAN_STOP_NAME,
			(char *)CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_navigator_ackerman_stop_message),
			(carmen_handler_t)navigator_ackerman_stop_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_compact_cost_map(
			NULL,
			(carmen_handler_t) map_server_compact_cost_map_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

//	carmen_behavior_selector_subscribe_goal_list_message(
//			NULL,
//			(carmen_handler_t) behaviour_selector_goal_list_message_handler,
//			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME,
			(char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_FMT,
			NULL, sizeof(carmen_navigator_ackerman_set_goal_message),
			(carmen_handler_t)navigator_ackerman_set_goal_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_grid_mapping_distance_map_subscribe_message(NULL,
			(carmen_handler_t) carmen_grid_mapping_distance_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
register_handlers()
{
	signal(SIGINT, signal_handler);

	if (!GlobalState::cheat)
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_goal_list_message(NULL, (carmen_handler_t) behaviour_selector_goal_list_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_road_profile_message(&goal_list_message, (carmen_handler_t) rddf_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ford_escape_subscribe_status_message(NULL, (carmen_handler_t) ford_escape_status_handler, CARMEN_SUBSCRIBE_LATEST);

	register_handlers_specific();
}


void
read_parameters_specific(int argc, char **argv)
{
	carmen_param_t optional_param_list[] = {
			{(char *)"rrt",	(char *)"use_obstacle_avoider", 	CARMEN_PARAM_ONOFF,		&GlobalState::use_obstacle_avoider, 	1, NULL},

			{(char *)"rrt",	(char *)"publish_tree",				CARMEN_PARAM_ONOFF,		&GlobalState::publish_tree,				1, NULL},
			{(char *)"rrt",	(char *)"reuse_last_path",			CARMEN_PARAM_ONOFF,		&GlobalState::reuse_last_path,			1, NULL},
			{(char *)"rrt",	(char *)"obstacle_cost_distance",	CARMEN_PARAM_DOUBLE,	&GlobalState::obstacle_cost_distance,	1, NULL}
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));
}


void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
			{(char *)"robot",	(char *)"length",								  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.length,								 			1, NULL},
			{(char *)"robot",	(char *)"width",								  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.width,								 			1, NULL},
			{(char *)"robot", 	(char *)"distance_between_rear_wheels",		  			CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_rear_wheels,			 		1, NULL},
			{(char *)"robot", 	(char *)"distance_between_front_and_rear_axles", 		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_front_and_rear_axles, 			1, NULL},
			{(char *)"robot", 	(char *)"distance_between_front_car_and_front_wheels",	CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_front_car_and_front_wheels,	1, NULL},
			{(char *)"robot", 	(char *)"distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_rear_car_and_rear_wheels,		1, NULL},
			{(char *)"robot", 	(char *)"max_velocity",						  			CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.max_v,									 		1, NULL},
			{(char *)"robot", 	(char *)"max_steering_angle",					  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.max_phi,								 		1, NULL},
			{(char *)"robot", 	(char *)"maximum_acceleration_forward",					CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_acceleration_forward,					1, NULL},
			{(char *)"robot", 	(char *)"maximum_acceleration_reverse",					CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_acceleration_reverse,					1, NULL},
			{(char *)"robot", 	(char *)"maximum_deceleration_forward",					CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_deceleration_forward,					1, NULL},
			{(char *)"robot", 	(char *)"maximum_deceleration_reverse",					CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_deceleration_reverse,					1, NULL},
			{(char *)"robot", 	(char *)"maximum_steering_command_rate",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_steering_command_rate,					1, NULL},
			{(char *)"robot", 	(char *)"understeer_coeficient",						CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.understeer_coeficient,							1, NULL}
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	GlobalState::param_max_vel = GlobalState::robot_config.max_v;

	//initialize default parameters values
	GlobalState::cheat = 0;

	carmen_param_t optional_param_list[] = {
			{(char *)"rrt",	(char *)"cheat",				CARMEN_PARAM_ONOFF,		&GlobalState::cheat,				1, NULL},
			{(char *)"rrt",	(char *)"show_debug_info",		CARMEN_PARAM_ONOFF,		&GlobalState::show_debug_info,		1, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));

	read_parameters_specific(argc, argv);

	carmen_param_t param_optional_list[] =
	{
			{(char *)"commandline", (char*)"update_lookup_table", CARMEN_PARAM_ONOFF, &update_lookup_table, 0, NULL}
	};

	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

	memset(&goal_list_message, 0, sizeof(goal_list_message));

	register_handlers();

	g_trajectory_lookup_table = new TrajectoryLookupTable(update_lookup_table);
	memset((void *) &tree, 0, sizeof(Tree));

	carmen_ipc_dispatch();
}

/*
 * model_predictive_planner_main.cpp
 *
 *  Created on: 04/12/2012
 *      Author: romulo
 */

#include <carmen/carmen.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/ford_escape_hybrid_interface.h>
#include <carmen/moving_objects_interface.h>

#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/grid_mapping.h>
#include <prob_map.h>

#include "model/global_state.h"
#include "model/rrt_node.h"
#include "model/pose.h"

#include "util.h"
#include "publisher_util.h"
#include "model_predictive_planner.h"

#define DIST_SQR(x1,y1,x2,y2) ((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

Tree tree; //tree rooted on robot
int g_teacher_mode = 0;
TrajectoryLookupTable *g_trajectory_lookup_table;
carmen_behavior_selector_road_profile_message goal_list_message;

static int update_lookup_table = 0;


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_model_predictive_rrt_path_message(list<RRT_Path_Edge> path, double timestamp)
{
	int i = 0;
	rrt_path_message msg;
	list<RRT_Path_Edge>::iterator it;

	msg.host  = carmen_get_host();
	msg.timestamp = timestamp;
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
publish_model_predictive_planner_motion_commands(vector<carmen_ackerman_path_point_t> path, double timestamp)
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
		commands[i].x = it->x;
		commands[i].y = it->y;
		commands[i].theta = it->theta;

		i++;
	}

	int num_commands = path.size();
	if (GlobalState::use_obstacle_avoider)
	{
		if (!g_teacher_mode)  // standard operation
			carmen_robot_ackerman_publish_motion_command(commands, num_commands, timestamp);
		else  // mode to prevent sending mpp commands to the rest of the control hierarchy and interfaces.
			carmen_robot_ackerman_publish_teacher_motion_command(commands, num_commands, timestamp);
	}
	else
		carmen_base_ackerman_publish_motion_command(commands, num_commands, timestamp);

	free(commands);
}


void
publish_path_follower_motion_commands(carmen_ackerman_motion_command_t *commands, int num_commands, double timestamp)
{
	if (GlobalState::use_obstacle_avoider)
	{
		if (!g_teacher_mode)  // standard operation
			carmen_robot_ackerman_publish_motion_command(commands, num_commands, timestamp);
		else  // mode to prevent sending mpp commands to the rest of the control hierarchy and interfaces.
			carmen_robot_ackerman_publish_teacher_motion_command(commands, num_commands, timestamp);
	}
	else
		carmen_base_ackerman_publish_motion_command(commands, num_commands, timestamp);
}


void
publish_path_follower_single_motion_command(double v, double phi, double timestamp)
{
	carmen_ackerman_motion_command_t commands[2];

	commands[0].v = v;
	commands[0].phi = phi;
	commands[0].time = 0.5;
	commands[1] = commands[0];
	publish_path_follower_motion_commands(commands, 2, timestamp);
}


void
publish_model_predictive_planner_single_motion_command(double v, double phi, double timestamp)
{
	vector<carmen_ackerman_path_point_t> path;

	carmen_ackerman_path_point_t traj;
	traj.v = v;
	traj.phi = phi;
	traj.time = 1.0;
	path.push_back(traj);
	path.push_back(traj);
	publish_model_predictive_planner_motion_commands(path, timestamp);

	publish_path_follower_single_motion_command(0.0, GlobalState::last_odometry.phi, timestamp);
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
		return;

	IPC_RETURN_TYPE err = IPC_OK;

	static bool first_time = true;
	if (first_time)
	{
		err = IPC_defineMsg(
				(char *)CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME,
				IPC_VARIABLE_LENGTH,
				(char *)CARMEN_NAVIGATOR_ACKERMAN_STATUS_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME);

		first_time = false;
	}

	carmen_navigator_ackerman_status_message msg;
	msg.autonomous = GlobalState::following_path;
	msg.goal_set   = GlobalState::goal_pose != NULL;

	if (msg.goal_set)
	{
		msg.goal.x = GlobalState::goal_pose->x;
		msg.goal.y = GlobalState::goal_pose->y;
		msg.goal.theta = GlobalState::goal_pose->theta;
		msg.goal.v = GlobalState::robot_config.max_v;
		msg.goal.phi = 0.0; // @@@ Alberto: terie que preencher isso...
	}
	else
	{
		msg.goal.theta = 0.0;
		msg.goal.y	   = 0.0;
		msg.goal.x	   = 0.0;
		msg.goal.v 	   = 0.0;
		msg.goal.phi = 0.0; // @@@ Alberto: terie que preencher isso...
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
		//		vector<carmen_ackerman_path_point_t> a;
		//		return a;
	}

	free_tree(tree);
	vector<vector<carmen_ackerman_path_point_t> > path = compute_path_to_goal(GlobalState::localizer_pose,
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
	publish_model_predictive_planner_single_motion_command(0.0, 0.0, carmen_get_time());
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
build_and_follow_path(double timestamp)
{
	list<RRT_Path_Edge> path_follower_path;

	if (GlobalState::goal_pose && (GlobalState::current_algorithm == CARMEN_BEHAVIOR_SELECTOR_RRT))
	{
		double distance_to_goal = sqrt(pow(GlobalState::goal_pose->x - GlobalState::localizer_pose->x, 2) + pow(GlobalState::goal_pose->y - GlobalState::localizer_pose->y, 2));
		// goal achieved!
		if (distance_to_goal < 0.5 && GlobalState::robot_config.max_v < 0.07 && GlobalState::last_odometry.v < 0.03)
		{
			publish_path_follower_single_motion_command(0.0, GlobalState::last_odometry.phi, timestamp);
		}
		else
		{
			vector<carmen_ackerman_path_point_t> path = compute_plan(&tree);
			if (tree.num_paths > 0 && path.size() > 0)
			{
				path_follower_path = build_path_follower_path(path);
				publish_model_predictive_rrt_path_message(path_follower_path, timestamp);
				publish_navigator_ackerman_plan_message(tree.paths[0], tree.paths_sizes[0]);

//				FILE *caco = fopen("caco2.txt", "a");
//				fprintf(caco, "%lf %lf %lf %d\n", GlobalState::last_odometry.v, GlobalState::robot_config.max_v,
//						path_follower_path.begin()->command.v,
//						GlobalState::behavior_selector_low_level_state);
//				fflush(caco);
//				fclose(caco);
			}
			//		else
				//			publish_path_follower_single_motion_command(0.0, GlobalState::last_odometry.phi, timestamp);
			//		{
			//			if (GlobalState::last_odometry.v == 0.0)
			//				publish_path_follower_single_motion_command(0.0, 0.0, timestamp);
			//			else
			//				publish_path_follower_single_motion_command(0.0, GlobalState::last_odometry.phi, timestamp);
			//		}
		}
		//publish_plan_tree_for_navigator_gui(tree);
		publish_navigator_ackerman_status_message();
	}
}


void
build_and_follow_path_new(double timestamp)
{
	if (GlobalState::goal_pose && (GlobalState::current_algorithm == CARMEN_BEHAVIOR_SELECTOR_RRT))
	{
		double distance_to_goal = sqrt(pow(GlobalState::goal_pose->x - GlobalState::localizer_pose->x, 2) + pow(GlobalState::goal_pose->y - GlobalState::localizer_pose->y, 2));
		// goal achieved!
		if (distance_to_goal < 0.5 && GlobalState::robot_config.max_v < 0.07 && GlobalState::last_odometry.v < 0.03)
		{
			publish_model_predictive_planner_single_motion_command(0.0, GlobalState::last_odometry.phi, timestamp);
		}
		else
		{
			vector<carmen_ackerman_path_point_t> path = compute_plan(&tree);
			if (tree.num_paths > 0 && path.size() > 0)
			{
				publish_model_predictive_planner_motion_commands(path, timestamp);
				publish_navigator_ackerman_plan_message(tree.paths[0], tree.paths_sizes[0]);
			}
			//		else
			//			publish_path_follower_single_motion_command(0.0, GlobalState::last_odometry.phi, timestamp);
		}
		//publish_plan_tree_for_navigator_gui(tree);
		publish_navigator_ackerman_status_message();
	}
}

/**
 * @brief      This function create discrete trajectory points for moving objects
 *
 * @param      new_msg             message containing the moving objects
 * @param  prediction_horizon  How many seconds in the future it will predict the trajectories
 * @param  time_interval       Interval of time between two points in the trajectory
 */
//static void
//construct_object_trajectories(carmen_moving_objects_point_clouds_message *new_msg, double prediction_horizon = 5, double time_interval = 0.15)
//{
//	GlobalState::moving_objects_trajectories.clear();
//
//	int num_objects = new_msg->num_point_clouds;
//	/* Reserve the memory beforehand to optimize construction */
//	GlobalState::moving_objects_trajectories.reserve(num_objects);
//	/* Precomputing this to use when moving object to car reference */
//    //TODO: check if this angle is correct or inverted
//	double cos_car = cos(-GlobalState::localizer_pose->theta);
//	double sin_car = sin(-GlobalState::localizer_pose->theta);
//
//	#ifdef DEBUG_OBJECTS_PLOT
//	carmen_ackerman_traj_point_t* all_traj = (carmen_ackerman_traj_point_t*)malloc(num_objects * static_cast<int>(prediction_horizon / time_interval) * sizeof(carmen_ackerman_traj_point_t));
//	#endif
//
//	for(int i = 0; i < num_objects; i++)
//	{
//		int num_trajectory_points = static_cast<int>(prediction_horizon / time_interval); //TODO: Checar esse time_interval
//		carmen_ackerman_traj_point_t *trajectory;
//		trajectory = (carmen_ackerman_traj_point_t*)malloc(num_trajectory_points * sizeof(carmen_ackerman_traj_point_t));
//		carmen_test_alloc(trajectory);
//
//		/* Now let's calculate the trajectory for the ith object*/
//		for(int j = 0; j < num_trajectory_points; j++)
//		{
//			carmen_ackerman_traj_point_t new_trajectory_point;
//			/* EXTREMELY SIMPLE MOTION SIMULATION USING SIMPLIFIED ACKERMAN MODEL */
//			/* X = X_0 + V * Cos(theta)*/
//			new_trajectory_point.x = new_msg->point_clouds[i].object_pose.x + (time_interval * j) * new_msg->point_clouds[i].linear_velocity * cos(new_msg->point_clouds[i].orientation);
//			/* Y = Y_0 + V * Sin(theta)*/
//			new_trajectory_point.y = new_msg->point_clouds[i].object_pose.y + (time_interval * j) * new_msg->point_clouds[i].linear_velocity * sin(new_msg->point_clouds[i].orientation);
//			/* Moving to car reference*/
//			if(GlobalState::localizer_pose)
//            {
//				new_trajectory_point.x = new_trajectory_point.x - GlobalState::localizer_pose->x;
//				new_trajectory_point.y = new_trajectory_point.y - GlobalState::localizer_pose->y;
//
//				//carmen_rotate_2d(&new_trajectory_point.x,&new_trajectory_point.y, GlobalState::localizer_pose->theta);
//				double x2 = new_trajectory_point.x;
//				new_trajectory_point.x = cos_car*(new_trajectory_point.x) - sin_car*(new_trajectory_point.y);
//  				new_trajectory_point.y = sin_car*x2 + cos_car*(new_trajectory_point.y);
//            }
//
//			/* We suppose the orientation remain constant */
//			new_trajectory_point.theta = new_msg->point_clouds[i].orientation - GlobalState::localizer_pose->theta;
//			new_trajectory_point.v = new_msg->point_clouds[i].linear_velocity;
//
//            /* We suppose no steering wheel angle, the car continue in the same direction forever */
//			new_trajectory_point.phi = 0.0;
//			//new_trajectory_point.time = new_msg->timestamp + j * time_interval;
//
//
//			trajectory[j] = new_trajectory_point;
//
//			#ifdef DEBUG_OBJECTS_PLOT
//			all_traj[i + num_objects*j] = new_trajectory_point;
//            carmen_rotate_2d(&all_traj[i + num_objects*j].x, &all_traj[i + num_objects*j].y, GlobalState::localizer_pose->theta);
//            all_traj[i + num_objects*j].x += GlobalState::localizer_pose->x;
//            all_traj[i + num_objects*j].y += GlobalState::localizer_pose->y;
//            all_traj[i + num_objects*j].theta += GlobalState::localizer_pose->theta;
//			#endif
//
//		} /* End of trajectory for one object */
//
//		GlobalState::moving_objects_trajectories.push_back(trajectory);
//
//	} /* End of pass through all objects*/
//
//	#ifdef DEBUG_OBJECTS_PLOT
//		carmen_rddf_publish_road_profile_around_end_point_message(all_traj, num_objects * static_cast<int>(prediction_horizon / time_interval));
//	#endif
//
//}


///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	Pose pose = Util::convert_to_pose(msg->globalpos);
	GlobalState::set_robot_pose(pose, msg->timestamp);

	if (GlobalState::use_mpc)
		build_and_follow_path_new(msg->timestamp);
	else
		build_and_follow_path(msg->timestamp);
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	GlobalState::last_odometry.v = msg->v;
	GlobalState::last_odometry.phi = msg->phi;

	Pose pose = Util::convert_to_pose(msg->truepose);
	GlobalState::set_robot_pose(pose, msg->timestamp);

	if (GlobalState::use_mpc)
		build_and_follow_path_new(msg->timestamp);
	else
		build_and_follow_path(msg->timestamp);
}


static void
navigator_ackerman_set_goal_message_handler(carmen_navigator_ackerman_set_goal_message *msg)
{
	// Na mensagem atual não é possível representar um goal nulo. Coordenadas do mundo são grandes.
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

//	GlobalState::robot_config.max_v = fmin(msg->goal_list->v, GlobalState::param_max_vel);
	double desired_v = fmin(msg->goal_list->v, GlobalState::param_max_vel);
	if (desired_v < GlobalState::robot_config.max_v)
		GlobalState::robot_config.max_v += (desired_v - GlobalState::robot_config.max_v) * 0.2;
	else
		GlobalState::robot_config.max_v += (desired_v - GlobalState::robot_config.max_v) * 0.1;
//	printf("v %lf\n", GlobalState::robot_config.max_v);

	GlobalState::set_goal_pose(goal_pose);
}


static void
behavior_selector_state_message_handler(carmen_behavior_selector_state_message *msg)
{
	GlobalState::behavior_selector_state = msg->state;
	GlobalState::behavior_selector_low_level_state = msg->low_level_state;
	GlobalState::current_algorithm = msg->algorithm;
}


//static void
//carmen_obstacle_distance_mapper_map_message_handler(carmen_obstacle_distance_mapper_map_message *message)
//{
//	GlobalState::distance_map = message;
//}


static void
carmen_obstacle_distance_mapper_compact_map_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	static carmen_obstacle_distance_mapper_compact_map_message *compact_distance_map = NULL;
	static carmen_obstacle_distance_mapper_map_message distance_map;

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

	GlobalState::distance_map = &distance_map;
}


static void
carmen_behaviour_selector_compact_lane_contents_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	if (GlobalState::distance_map)
		carmen_obstacle_distance_mapper_overwrite_distance_map_message_with_compact_distance_map(GlobalState::distance_map, message);
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
lane_message_handler(/*carmen_behavior_selector_road_profile_message *message*/)
{
	//	printf("RDDF NUM POSES: %d \n", message->number_of_poses);
	//
	//	for (int i = 0; i < message->number_of_poses; i++)
	//	{
	//		printf("RDDF %d: x  = %lf, y = %lf , theta = %lf\n", i, message->poses[i].x, message->poses[i].y, message->poses[i].theta);
	//		getchar();
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
	system("pkill gnuplot");
	carmen_ipc_disconnect();
	exit(1);
}
//
//static void
//carmen_moving_objects_point_clouds_message_handler(carmen_moving_objects_point_clouds_message* msg)
//{
//	GlobalState::objects_message = msg;
//	GlobalState::moving_objects_initialized = true;
//	construct_object_trajectories(msg);
//}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
register_handlers()
{
	signal(SIGINT, signal_handler);

	if (!GlobalState::use_truepos)
	{
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
		carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
	}
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_goal_list_message(NULL, (carmen_handler_t) behaviour_selector_goal_list_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME, (char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_FMT,
			&goal_list_message, sizeof (carmen_behavior_selector_road_profile_message), (carmen_handler_t) lane_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ford_escape_subscribe_status_message(NULL, (carmen_handler_t) ford_escape_status_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *) CARMEN_NAVIGATOR_ACKERMAN_GO_NAME,
			(char *) CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_navigator_ackerman_go_message),
			(carmen_handler_t)navigator_ackerman_go_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *) CARMEN_NAVIGATOR_ACKERMAN_STOP_NAME,
			(char *) CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_navigator_ackerman_stop_message),
			(carmen_handler_t)navigator_ackerman_stop_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *) CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME,
			(char *) CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_FMT,
			NULL, sizeof(carmen_navigator_ackerman_set_goal_message),
			(carmen_handler_t)navigator_ackerman_set_goal_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

//	carmen_moving_objects_point_clouds_subscribe_message(NULL,
//			(carmen_handler_t) carmen_moving_objects_point_clouds_message_handler,
//			CARMEN_SUBSCRIBE_LATEST);

//	carmen_obstacle_distance_mapper_subscribe_message(NULL,
//			(carmen_handler_t) carmen_obstacle_distance_mapper_map_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_obstacle_distance_mapper_subscribe_compact_map_message(NULL,
			(carmen_handler_t) carmen_obstacle_distance_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behaviour_selector_subscribe_compact_lane_contents_message(NULL,
			(carmen_handler_t) carmen_behaviour_selector_compact_lane_contents_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
read_parameters_specific(int argc, char **argv)
{
	carmen_param_t optional_param_list[] = {
		{(char *) "model_predictive_planner", 	(char *) "obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.model_predictive_planner_obstacles_safe_distance, 1, NULL},
        {(char *) "model_predictive_planner", 	(char *) "obstacles_safe_length_distance", CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.model_predictive_planner_obstacles_safe_length_distance, 1, NULL},
        {(char *) "model_predictive_planner", 	(char *) "max_square_distance_to_lane", CARMEN_PARAM_DOUBLE, &GlobalState::max_square_distance_to_lane, 1, NULL},
		{(char *) "obstacle_avoider", 			(char *) "obstacles_safe_distance", CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.obstacle_avoider_obstacles_safe_distance, 1, NULL},
		{(char *) "rrt",	(char *) "use_obstacle_avoider", 	CARMEN_PARAM_ONOFF,		&GlobalState::use_obstacle_avoider, 	1, NULL},
		{(char *) "rrt",	(char *) "use_mpc",					CARMEN_PARAM_ONOFF,		&GlobalState::use_mpc, 					0, NULL},
		{(char *) "rrt",	(char *) "publish_tree",			CARMEN_PARAM_ONOFF,		&GlobalState::publish_tree,				1, NULL},
		{(char *) "rrt",	(char *) "reuse_last_path",			CARMEN_PARAM_ONOFF,		&GlobalState::reuse_last_path,			1, NULL},
		{(char *) "rrt",	(char *) "obstacle_cost_distance",	CARMEN_PARAM_DOUBLE,	&GlobalState::obstacle_cost_distance,	1, NULL}
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));
}


void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
		{(char *) "robot",	(char *) "length",								  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.length,								 			1, NULL},
		{(char *) "robot",	(char *) "width",								  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.width,								 			1, NULL},
		{(char *) "robot", 	(char *) "distance_between_rear_wheels",		  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_rear_wheels,			 		1, NULL},
		{(char *) "robot", 	(char *) "distance_between_front_and_rear_axles", 		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_front_and_rear_axles, 			1, NULL},
		{(char *) "robot", 	(char *) "distance_between_front_car_and_front_wheels",	CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_front_car_and_front_wheels,	1, NULL},
		{(char *) "robot", 	(char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_rear_car_and_rear_wheels,		1, NULL},
		{(char *) "robot", 	(char *) "max_velocity",						  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.max_v,									 		1, NULL},
		{(char *) "robot", 	(char *) "max_steering_angle",					  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.max_phi,								 		1, NULL},
		{(char *) "robot", 	(char *) "maximum_acceleration_forward",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_acceleration_forward,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_acceleration_reverse",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_acceleration_reverse,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_deceleration_forward",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_deceleration_forward,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_deceleration_reverse",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_deceleration_reverse,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_steering_command_rate",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_steering_command_rate,					1, NULL},
		{(char *) "robot", 	(char *) "understeer_coeficient",						CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.understeer_coeficient,							1, NULL},
		{(char *) "robot", 	(char *) "max_centripetal_acceleration",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_max_centripetal_acceleration,							1, NULL},
		{(char *) "rddf",   (char *) "source_tracker", 								CARMEN_PARAM_ONOFF,  &GlobalState::use_tracker_goal_and_lane,									0, NULL},
		{(char *) "behavior_selector", (char *) "goal_source_path_planner", 		CARMEN_PARAM_ONOFF,  &GlobalState::use_path_planner, 											0, NULL},
		{(char *) "behavior_selector", (char *) "use_truepos", 						CARMEN_PARAM_ONOFF,  &GlobalState::use_truepos, 												0, NULL}
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	GlobalState::param_max_vel = GlobalState::robot_config.max_v;

	read_parameters_specific(argc, argv);

	carmen_param_t param_optional_list[] =
	{
		{(char *) "commandline", (char *) "update_lookup_table", CARMEN_PARAM_ONOFF, &update_lookup_table, 0, NULL},
		{(char *) "commandline", (char *) "teacher_mode", CARMEN_PARAM_ONOFF, &g_teacher_mode, 0, NULL}
	};

	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));
}

//extern carmen_mapper_virtual_laser_message virtual_laser_message;
//#define MAX_VIRTUAL_LASER_SAMPLES 10000

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

//	memset(&virtual_laser_message, 0, sizeof(carmen_mapper_virtual_laser_message));
//	virtual_laser_message.positions = (carmen_position_t *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(carmen_position_t));
//	virtual_laser_message.colors = (char *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(char));
//	virtual_laser_message.host = carmen_get_host();

	carmen_ipc_dispatch();
}

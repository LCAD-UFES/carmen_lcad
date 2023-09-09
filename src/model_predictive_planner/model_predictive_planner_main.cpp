#include <carmen/carmen.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/ford_escape_hybrid_interface.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/task_manager_interface.h>

#include <carmen/voice_interface_messages.h>
#include <carmen/voice_interface_interface.h>

#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/grid_mapping.h>
#include <prob_map.h>

#include "model/global_state.h"
#include "rrt_node.h"
#include "pose.h"

#include "util.h"
#include "publisher_util.h"
#include "model_predictive_planner.h"
#include "model_predictive_planner_interface.h"

#include <carmen/collision_detection.h>
#include <car_model.h>


//#define save_rddf_to_file

Tree tree; //tree rooted on robot
int g_teacher_mode = 0;
//TrajectoryLookupTable *g_trajectory_lookup_table;
carmen_behavior_selector_path_goals_and_annotations_message *path_goals_and_annotations_message = NULL;

static int update_lookup_table = 0;

static int argc_global;
static char **argv_global;

int use_unity_simulator = 0;

extern double desired_v;

//static void
//print_path_(vector<carmen_robot_and_trailer_path_point_t> path)
//{
//	for (unsigned int i = 0; (i < path.size()) && (i < 15); i++)
//		printf("v %5.3lf, phi %5.3lf, t %5.3lf, x %5.3lf, y %5.3lf, theta %5.3lf\n",
//				path[i].v, path[i].phi, path[i].time,
//				path[i].x, path[i].y,
//				path[i].theta);
//
//	printf("\n");
//	fflush(stdout);
//}

double original_model_predictive_planner_obstacles_safe_distance;

double voice_interface_max_vel = 0.0;


vector<carmen_robot_and_trailers_path_point_t>
smooth_short_path(vector<carmen_robot_and_trailers_path_point_t> &original_path)
{
	vector<carmen_robot_and_trailers_path_point_t> path = original_path;

	static double stable_phi = 0.0;	// Ultimo phi de um path nao short
	double distance_travelled = 0.0;
	if (path.size() > 1)
	{
		distance_travelled = DIST2D(path[0], path[path.size() - 1]);
		if (distance_travelled < 0.6)
		{
			for (unsigned int j = 0; j < path.size(); j++)
				original_path[j].phi = stable_phi;
		}
		else
			stable_phi = path[0].phi;
	}

	path = original_path;

	return (path);
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_model_predictive_planner_motion_commands(vector<carmen_robot_and_trailers_path_point_t> path, double timestamp)
{
	if (!GlobalState::following_path)
		return;

	carmen_robot_and_trailers_motion_command_t *commands =
			(carmen_robot_and_trailers_motion_command_t *) (malloc(path.size() * sizeof(carmen_robot_and_trailers_motion_command_t)));
	int i = 0;
	for (std::vector<carmen_robot_and_trailers_path_point_t>::iterator it = path.begin();	it != path.end(); ++it)
	{
		commands[i].v = it->v;
		commands[i].phi = it->phi;
		commands[i].time = it->time;
		commands[i].x = it->x;
		commands[i].y = it->y;
		commands[i].theta = it->theta;
		commands[i].num_trailers = it->num_trailers;
		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			commands[i].trailer_theta[z] = it->trailer_theta[z];

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


vector<carmen_robot_and_trailers_path_point_t>
apply_robot_delays(vector<carmen_robot_and_trailers_path_point_t> &original_path)
{
	// Velocity delay
	vector<carmen_robot_and_trailers_path_point_t> path = original_path;
	double time_delay = 0.0;
	double distance_travelled = 0.0;
	int i = 0;
	while ((time_delay < GlobalState::robot_velocity_delay) && (path.size() > 1))
	{
		time_delay += path[0].time;
		distance_travelled += DIST2D(path[0], path[1]);
		path.erase(path.begin());
		i++;
	}

	while ((distance_travelled < GlobalState::robot_min_v_distance_ahead) && (path.size() > 1))
	{
		distance_travelled += DIST2D(path[0], path[1]);
		path.erase(path.begin());
		i++;
	}

	for (unsigned int j = 0; j < path.size(); j++)
		original_path[j].v = path[j].v;

	int size_decrease_due_to_velocity_delay = i;

	// Steering delay
	path = original_path;
	time_delay = 0.0;
	distance_travelled = 0.0;
	i = 0;
	while ((time_delay < GlobalState::robot_steering_delay) && (path.size() > 1))
	{
		time_delay += path[0].time;
		distance_travelled += DIST2D(path[0], path[1]);
		path.erase(path.begin());
		i++;
	}

	while ((distance_travelled < GlobalState::robot_min_s_distance_ahead) && (path.size() > 1))
	{
		distance_travelled += DIST2D(path[0], path[1]);
		path.erase(path.begin());
		i++;
	}

	for (unsigned int j = 0; j < path.size(); j++)
		original_path[j].phi = path[j].phi;

	int size_decrease_due_to_steering_delay = i;

	int size_decrease = (size_decrease_due_to_velocity_delay > size_decrease_due_to_steering_delay) ?
							size_decrease_due_to_velocity_delay : size_decrease_due_to_steering_delay;

	original_path.erase(original_path.begin() + original_path.size() - size_decrease, original_path.end());

	path = original_path;

	return (path);
}


void
publish_robot_ackerman_motion_commands_eliminating_path_follower(vector<carmen_robot_and_trailers_path_point_t> &original_path, double timestamp)
{
//	vector<carmen_robot_and_trailers_path_point_t> path = smooth_short_path(original_path);	// A plicacao dos atrazos do robo agora são na saida do obstacle_avoider
	vector<carmen_robot_and_trailers_path_point_t> path = original_path;//apply_robot_delays(original_path);	// A plicacao dos atrazos do robo agora são na saida do obstacle_avoider
//	print_path_(path);
	publish_model_predictive_planner_motion_commands(path, timestamp);
}


void
publish_model_predictive_planner_rrt_path_message(list<RRT_Path_Edge> path, double timestamp)
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
//		msg.path[i].p1.trailer_theta[0] = it->p1.pose.beta;
		for (size_t j = 0; j < MAX_NUM_TRAILERS; j++)
			msg.path[i].p1.trailer_theta[j] = it->p1.pose.trailer_theta[j];
//			msg.path[i].p1.trailer_theta[j] = it->p1.pose.beta; // Apenas para não deixar valor aleatório nos outros trailers


		msg.path[i].p1.v = it->p1.v_and_phi.v;
		msg.path[i].p1.phi = it->p1.v_and_phi.phi;

		msg.path[i].p2.x = it->p2.pose.x;
		msg.path[i].p2.y = it->p2.pose.y;
//		msg.path[i].p2.trailer_theta[0] = it->p2.pose.beta;
		for (size_t j = 0; j < MAX_NUM_TRAILERS; j++)
			msg.path[i].p2.trailer_theta[j] = it->p2.pose.trailer_theta[j];
//			msg.path[i].p2.trailer_theta[j] = it->p2.pose.beta;

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
publish_path_follower_motion_commands(carmen_robot_and_trailers_motion_command_t *commands, int num_commands, double timestamp)
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
	carmen_robot_and_trailers_motion_command_t commands[2];

	commands[0].v = v;
	commands[0].phi = phi;
	commands[0].time = 0.5;
	commands[1] = commands[0];
	publish_path_follower_motion_commands(commands, 2, timestamp);
}


void
publish_model_predictive_planner_single_motion_command(double v, double phi, double timestamp)
{
	vector<carmen_robot_and_trailers_path_point_t> path;

	carmen_robot_and_trailers_path_point_t traj;
	traj.v = v;
	traj.phi = phi;
	traj.time = 1.0;
	traj.x = GlobalState::localizer_pose->x;
	traj.y = GlobalState::localizer_pose->y;
	traj.theta = GlobalState::localizer_pose->theta;
	traj.num_trailers = GlobalState::localizer_pose->num_trailers;
	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		traj.trailer_theta[z] =	GlobalState::localizer_pose->trailer_theta[z];

	path.push_back(traj);
	path.push_back(traj);
	publish_model_predictive_planner_motion_commands(path, timestamp);

//	publish_path_follower_single_motion_command(0.0, GlobalState::last_odometry.phi, timestamp);
	publish_path_follower_single_motion_command(0.0, phi, timestamp);
}


void
publish_navigator_ackerman_plan_message(carmen_robot_and_trailers_traj_point_t *path, int path_size)
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
//		msg.goal.trailer_theta[0] = GlobalState::goal_pose->beta;
		for (size_t j = 0; j < MAX_NUM_TRAILERS; j++)
			msg.goal.trailer_theta[j] = GlobalState::goal_pose->trailer_theta[j];
//			msg.goal.trailer_theta[j] = GlobalState::goal_pose->beta;


		msg.goal.v = (path_goals_and_annotations_message != NULL)? path_goals_and_annotations_message->goal_list->v: GlobalState::robot_config.max_v;
		msg.goal.phi = 0.0; // @@@ Alberto: teria que preencher isso...
	}
	else
	{
		msg.goal.theta = 0.0;
		msg.goal.y	   = 0.0;
		msg.goal.x	   = 0.0;
		msg.goal.v 	   = 0.0;
		msg.goal.phi = 0.0; // @@@ Alberto: teria que preencher isso...
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
copy_path_to_traj(carmen_robot_and_trailers_traj_point_t *traj, vector<carmen_robot_and_trailers_path_point_t> path)
{
	int i = 0;
	for (std::vector<carmen_robot_and_trailers_path_point_t>::iterator it = path.begin(); it != path.end(); ++it, ++i)
	{
		traj[i].x = it->x;
		traj[i].y = it->y;
		traj[i].theta = it->theta;
		traj[i].num_trailers = it->num_trailers;
		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			traj[i].trailer_theta[z] = it->trailer_theta[z];
		traj[i].v = it->v;
		traj[i].phi = it->phi;
	}
}


vector<carmen_robot_and_trailers_path_point_t>
compute_plan(Tree *tree)
{
	if (!path_goals_and_annotations_message || (path_goals_and_annotations_message->number_of_poses == 0))
	{
		tree->num_paths = 0;
		tree->paths = NULL;
		vector<carmen_robot_and_trailers_path_point_t> voidVector;

		return (voidVector);
	}

	free_tree(tree);
	vector<vector<carmen_robot_and_trailers_path_point_t> > path = compute_path_to_goal(GlobalState::localizer_pose,
			GlobalState::goal_pose, GlobalState::last_odometry, GlobalState::robot_config.max_v, path_goals_and_annotations_message);

	if (path.size() == 0)
	{
		tree->num_paths = 0;
		tree->paths = NULL;
		vector<carmen_robot_and_trailers_path_point_t> voidVector;

		return (voidVector);
	}
	else
	{
		tree->num_paths = path.size();
		tree->paths = (carmen_robot_and_trailers_traj_point_t **) malloc(tree->num_paths * sizeof(carmen_robot_and_trailers_traj_point_t *));
		tree->paths_sizes = (int *) malloc(tree->num_paths * sizeof(int));

		for (unsigned int i = 0; i < path.size(); i++)
		{
			tree->paths[i] = (carmen_robot_and_trailers_traj_point_t *) malloc(path[i].size() * sizeof(carmen_robot_and_trailers_traj_point_t));
			copy_path_to_traj(tree->paths[i], path[i]);
			tree->paths_sizes[i] = path[i].size();
		}

		if (!GlobalState::last_plan_pose)
			GlobalState::last_plan_pose = (carmen_robot_and_trailers_pose_t *) malloc(sizeof(carmen_robot_and_trailers_pose_t));
		*GlobalState::last_plan_pose = *GlobalState::localizer_pose;

		return (path[0]);
	}
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
build_path_follower_path(vector<carmen_robot_and_trailers_path_point_t> path)
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
//		path_edge.p1.pose.beta = path[i].trailer_theta[0];
		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			path_edge.p1.pose.trailer_theta[z] = path[i].trailer_theta[z];

		path_edge.p1.v_and_phi.v = path[i].v;
		path_edge.p1.v_and_phi.phi = path[i].phi;

		path_edge.p2.pose.x = path[i + 1].x;
		path_edge.p2.pose.y = path[i + 1].y;
		path_edge.p2.pose.theta = path[i + 1].theta;
//		path_edge.p2.pose.beta = path[i + 1].trailer_theta[0];
		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			path_edge.p2.pose.trailer_theta[z] = path[i + 1].trailer_theta[z];

		path_edge.p1.v_and_phi.v = path[i + 1].v;
		path_edge.p1.v_and_phi.phi = path[i + 1].phi;

		path_edge.command.v = path[i + 1].v;
		path_edge.command.phi = path[i + 1].phi;
		path_edge.time = path[i].time;

		path_follower_path.push_back(path_edge);
	}

	return (path_follower_path);
}


bool
goal_crossed()
{
	bool reverse_planning;
	if ((GlobalState::robot_config.max_v < 0.0) && GlobalState::reverse_driving_flag)
		reverse_planning = true;
	else
		reverse_planning = false;

	if (reverse_planning)
	{
		double angle_wrt_goal = carmen_normalize_theta(ANGLE2D_P(GlobalState::goal_pose, GlobalState::localizer_pose) - GlobalState::localizer_pose->theta);
		if (angle_wrt_goal > M_PI_2)
			return (true);
		else
			return (false);
	}
	else
	{
		double angle_wrt_goal = carmen_normalize_theta(ANGLE2D_P(GlobalState::localizer_pose, GlobalState::goal_pose) - GlobalState::localizer_pose->theta);
		if (angle_wrt_goal > M_PI_2)
			return (true);
		else
			return (false);
	}
}


void
build_and_follow_path(double timestamp)
{
	list<RRT_Path_Edge> path_follower_path;
	static double last_phi = 0.0;
//	static int count = 0;
//
//	if ((count++ % 40) != 0)
//		return;

	if (GlobalState::goal_pose && (GlobalState::route_planner_state != PLANNING_FROM_POSE_TO_LANE))
	{
		double distance_to_goal = DIST2D_P(GlobalState::goal_pose, GlobalState::localizer_pose);
		if (((distance_to_goal < (1.0 * (GlobalState::robot_config.distance_between_front_and_rear_axles / 2.625))) && (fabs(GlobalState::robot_config.max_v) < 0.07) && (fabs(GlobalState::last_odometry.v) < 0.03)))// ||
//			((distance_to_goal < 0.3) && (fabs(GlobalState::robot_config.max_v) < 0.07) && (fabs(GlobalState::last_odometry.v) < 0.5) &&
//					(path_goals_and_annotations_message->number_of_poses == 1)))
		{
//			printf("*np %d, gls %d, dtg %5.2lf, max_v %5.2lf, v %5.2lf\n",
//					path_goals_and_annotations_message->number_of_poses, path_goals_and_annotations_message->goal_list_size,
//					distance_to_goal, GlobalState::robot_config.max_v, GlobalState::last_odometry.v);
//			fflush(stdout);

			GlobalState::robot_config.max_v = 0.0;
			if (GlobalState::following_path)
			{
				last_phi *= 0.95;
				publish_path_follower_single_motion_command(0.0, last_phi, timestamp);
			}
			else	// Stop button
			{
				last_phi *= 0.9;
				publish_path_follower_single_motion_command(0.0, last_phi, timestamp);
			}
		}
		else
		{
			vector<carmen_robot_and_trailers_path_point_t> path = compute_plan(&tree);
			if (!GlobalState::path_has_collision_or_phi_exceeded && (tree.num_paths > 0) && (path.size() > 0))
			{
				if (GlobalState::eliminate_path_follower)
					publish_robot_ackerman_motion_commands_eliminating_path_follower(path, timestamp);
				path_follower_path = build_path_follower_path(path);
				publish_model_predictive_planner_rrt_path_message(path_follower_path, timestamp);
//				carmen_model_predictive_planner_publish_motion_plan_message(tree.paths[0], tree.paths_sizes[0]);
			}
			else if (GlobalState::path_has_collision_or_phi_exceeded && (path.size() > 0) && (fabs(GlobalState::last_odometry.v) < 0.03))
			{
				publish_path_follower_single_motion_command(0.0, path[0].phi, timestamp);
			}
			else
			{
				GlobalState::robot_config.max_v = 0.0;
				if (GlobalState::following_path)
				{
					last_phi *= 0.95;
					publish_path_follower_single_motion_command(0.0, last_phi, timestamp);
				}
				else	// Stop button
				{
					last_phi *= 0.9;
					publish_path_follower_single_motion_command(0.0, last_phi, timestamp);
				}
			}
//			printf(" np %d, gls %d, dtg %5.2lf, max_v %5.3lf, v %5.3lf, ps %d\n",
//					path_goals_and_annotations_message->number_of_poses, path_goals_and_annotations_message->goal_list_size,
//					distance_to_goal, GlobalState::robot_config.max_v, GlobalState::last_odometry.v, (int) path.size());
//			print_path_(path);
//			fflush(stdout);

			last_phi = GlobalState::last_odometry.phi;
		}
		publish_navigator_ackerman_status_message();
//		publish_plan_tree_for_navigator_gui(tree);
	}
}


void
build_and_follow_path_new(double timestamp)
{
	static double last_phi = 0.0;

	if (GlobalState::goal_pose)
	{
		double distance_to_goal = sqrt(pow(GlobalState::goal_pose->x - GlobalState::localizer_pose->x, 2) + pow(GlobalState::goal_pose->y - GlobalState::localizer_pose->y, 2));
		// goal achieved!
		if (distance_to_goal < 0.3 && GlobalState::robot_config.max_v < 0.07 && GlobalState::last_odometry.v < 0.03)
		{
			if (GlobalState::following_path)
			{
				last_phi *= 0.95;
				publish_path_follower_single_motion_command(0.0, last_phi, timestamp);
			}
			else	// Stop button
			{
				last_phi *= 0.9;
				publish_path_follower_single_motion_command(0.0, last_phi, timestamp);
			}
		}
		else
		{
			vector<carmen_robot_and_trailers_path_point_t> path = compute_plan(&tree);
			if (tree.num_paths > 0 && path.size() > 0)
			{
				publish_model_predictive_planner_motion_commands(path, timestamp);
				carmen_model_predictive_planner_publish_motion_plan_message(tree.paths[0], tree.paths_sizes[0]);
			}
			last_phi = GlobalState::last_odometry.phi;
		}
		publish_navigator_ackerman_status_message();
//		publish_plan_tree_for_navigator_gui(tree);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
//	static double previous_timestamp = 0.0;
//	if (previous_timestamp)
//	{
//		printf("globalpos %lf, %lf\n", msg->timestamp, msg->timestamp - previous_timestamp);
//		fflush(stdout);
//	}
//	previous_timestamp = msg->timestamp;

	if (!GlobalState::localizer_pose)
		GlobalState::localizer_pose = (carmen_robot_and_trailers_pose_t *) malloc(sizeof(carmen_robot_and_trailers_pose_t));

	*GlobalState::localizer_pose = {msg->globalpos.x, msg->globalpos.y, msg->globalpos.theta,  msg->num_trailers, {0.0}}; //Adicionar num_trailers

	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		GlobalState::localizer_pose->trailer_theta[z] = msg->trailer_theta[z];

//	double delta_t = 0.15;
//	double distance_traveled = 0.0;
//	carmen_robot_and_trailers_traj_point_t robot_state = {GlobalState::localizer_pose->x, GlobalState::localizer_pose->y, GlobalState::localizer_pose->theta,
//			GlobalState::localizer_pose->num_trailers, {
//					GlobalState::localizer_pose->trailer_theta[0],
//					GlobalState::localizer_pose->trailer_theta[1],
//					GlobalState::localizer_pose->trailer_theta[2],
//					GlobalState::localizer_pose->trailer_theta[3],
//					GlobalState::localizer_pose->trailer_theta[4]},
//					GlobalState::last_odometry.v, GlobalState::last_odometry.phi};
//
//	robot_state = carmen_libcarmodel_recalc_pos_ackerman(robot_state, GlobalState::last_odometry.v, GlobalState::last_odometry.phi, delta_t,
//						&distance_traveled, delta_t, GlobalState::robot_config, GlobalState::semi_trailer_config);
//
//	*GlobalState::localizer_pose = {robot_state.x, robot_state.y, robot_state.theta, robot_state.num_trailers, {
//			robot_state.trailer_theta[0],
//			robot_state.trailer_theta[1],
//			robot_state.trailer_theta[2],
//			robot_state.trailer_theta[3],
//			robot_state.trailer_theta[4]}};

	if (GlobalState::use_mpc)
		build_and_follow_path_new(msg->timestamp);
	else
		build_and_follow_path(msg->timestamp);

	if (msg->semi_trailer_type != GlobalState::semi_trailer_config.num_semi_trailers)
	{
		carmen_task_manager_read_semi_trailer_parameters(&GlobalState::semi_trailer_config, argc_global, argv_global, msg->semi_trailer_type);
		carmen_collision_detection_set_semi_trailer_type(GlobalState::semi_trailer_config.num_semi_trailers);
	}
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	GlobalState::last_odometry.v = msg->v;
	GlobalState::last_odometry.phi = msg->phi;

	if (!GlobalState::localizer_pose)
			GlobalState::localizer_pose = (carmen_robot_and_trailers_pose_t *) malloc(sizeof(carmen_robot_and_trailers_pose_t));

	*GlobalState::localizer_pose = {msg->truepose.x, msg->truepose.y, msg->truepose.theta, msg->num_trailers, {0.0}};
	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		GlobalState::localizer_pose->trailer_theta[z] = msg->trailer_theta[z];

	if (GlobalState::use_mpc)
		build_and_follow_path_new(msg->timestamp);
	else
		build_and_follow_path(msg->timestamp);
}


static void
path_goals_and_annotations_message_handler(carmen_behavior_selector_path_goals_and_annotations_message *msg)
{
//	static double previous_timestamp = 0.0;
//	if (previous_timestamp)
//	{
//		printf("path %lf, %lf\n", msg->timestamp, msg->timestamp - previous_timestamp);
//		fflush(stdout);
//	}
//	previous_timestamp = msg->timestamp;

	path_goals_and_annotations_message = msg;

	Pose goal_pose;

	if ((msg->goal_list_size <= 0) || !msg->goal_list || !GlobalState::localizer_pose)
	{
		printf("Empty goal list or localize not received\n");
		return;
	}

	GlobalState::last_goal = (msg->goal_list_size == 1)? true: false;

	goal_pose.x = msg->goal_list[0].x;
	goal_pose.y = msg->goal_list[0].y;
	goal_pose.theta = carmen_normalize_theta(msg->goal_list[0].theta);
//	goal_pose.beta = msg->goal_list[0].trailer_theta[0];

	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		goal_pose.trailer_theta[z] = msg->goal_list[0].trailer_theta[z];

	if (GlobalState::reverse_driving_flag)
	{
		if (msg->goal_list[0].v < 0.0)
		{
//			if (GlobalState::robot_config.max_v > 0.0)
//				GlobalState::robot_config.max_v = GlobalState::param_max_vel_reverse;

			desired_v = fmax(msg->goal_list[0].v, GlobalState::param_max_vel_reverse);

			if (desired_v < GlobalState::robot_config.max_v)
				GlobalState::robot_config.max_v += (desired_v - GlobalState::robot_config.max_v) * 0.5;
			else
				GlobalState::robot_config.max_v += (desired_v - GlobalState::robot_config.max_v) * 0.1;
		}
		else
		{
//			if (GlobalState::robot_config.max_v < 0.0)	// Acaba de pedir inversao da velovidade de negativa para positiva
//				GlobalState::robot_config.max_v = GlobalState::param_max_vel;

			desired_v = fmin(msg->goal_list[0].v, GlobalState::param_max_vel);
			if (desired_v > GlobalState::robot_config.max_v)
				GlobalState::robot_config.max_v += (desired_v - GlobalState::robot_config.max_v) * 0.5;
			else
				GlobalState::robot_config.max_v += (desired_v - GlobalState::robot_config.max_v) * 0.1;
		}
	}
	else
	{
		desired_v = fmin(msg->goal_list[0].v, GlobalState::param_max_vel);
		if (desired_v > GlobalState::robot_config.max_v)
			GlobalState::robot_config.max_v += (desired_v - GlobalState::robot_config.max_v) * 0.5;
		else
			GlobalState::robot_config.max_v += (desired_v - GlobalState::robot_config.max_v) * 0.1;
	}

//	printf("t %lf, goal_v %lf, v %lf\n", msg->timestamp, msg->goal_list[0].v, GlobalState::last_odometry.v);

//	if (fabs(GlobalState::robot_config.max_v) < 0.0005)	// Para evitar aproximacoes que nunca chegam a zero.
//		GlobalState::robot_config.max_v = 0.0;

//	printf("*target_v %lf\n", GlobalState::robot_config.max_v);

	GlobalState::set_goal_pose(goal_pose);
}


static void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *msg)
{
	GlobalState::last_odometry.v = msg->v;
	GlobalState::last_odometry.phi = msg->phi;

	if (fabs(msg->v) < GlobalState::eliminate_path_follower_transition_v)
		GlobalState::eliminate_path_follower = 1;
	else
		GlobalState::eliminate_path_follower = 0;
}


static void
behavior_selector_state_message_handler(carmen_behavior_selector_state_message *msg)
{
	GlobalState::behavior_selector_task = msg->task;
	GlobalState::behavior_selector_low_level_state = msg->low_level_state;
	GlobalState::current_algorithm = msg->algorithm;
	GlobalState::route_planner_state = msg->route_planner_state;
	GlobalState::offroad_planner_request = msg->offroad_planner_request;

	if (msg->low_level_state_flags & CARMEN_BEHAVIOR_SELECTOR_WITHIN_NARROW_PASSAGE)
		GlobalState::robot_config.model_predictive_planner_obstacles_safe_distance = 0.0;
	else
		GlobalState::robot_config.model_predictive_planner_obstacles_safe_distance = original_model_predictive_planner_obstacles_safe_distance;

	if (msg->low_level_state_flags & CARMEN_BEHAVIOR_SELECTOR_ENGAGE_COLLISION_GEOMETRY)
		carmen_collision_detection_set_robot_collision_config(ENGAGE_GEOMETRY);
	else
		carmen_collision_detection_set_robot_collision_config(DEFAULT_GEOMETRY);
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


void
ford_escape_status_handler(carmen_ford_escape_status_message *msg)
{
	//TODO tratar tambem comandos de ultrapassagem (ou talvez tratar no behavior selector)
	GlobalState::ford_escape_status.g_XGV_turn_signal = msg->g_XGV_turn_signal;
	//Tratando se o navegador esta em no modo real ou em modo simulacao
	GlobalState::ford_escape_online = true;
}


void
carmen_voice_interface_command_message_handler(carmen_voice_interface_command_message *message)
{
	if (message->command_id == SET_SPEED)
	{
		if (strcmp(message->command, "MAX_SPEED") == 0)
			GlobalState::robot_config.max_v = GlobalState::param_max_vel = voice_interface_max_vel;
		else if (strcmp(message->command, "0.0") == 0)
			GlobalState::robot_config.max_v = GlobalState::robot_config.max_v; // não faz nada, até ter implementação melhor do soft stop
		else
			GlobalState::robot_config.max_v = GlobalState::param_max_vel = strtod(message->command, NULL);
	}
}


static void
signal_handler(int sig)
{
	printf("Signal %d received, exiting program ...\n", sig);
	system("pkill gnuplot");
	carmen_ipc_disconnect();
	exit(1);
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
register_handlers()
{
	if (!GlobalState::use_truepos)
	{
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
		carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
	}
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_path_goals_and_annotations_message(NULL, (carmen_handler_t) path_goals_and_annotations_message_handler, CARMEN_SUBSCRIBE_LATEST);

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

//	carmen_subscribe_message(
//			(char *) CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME,
//			(char *) CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_FMT,
//			NULL, sizeof(carmen_navigator_ackerman_set_goal_message),
//			(carmen_handler_t)navigator_ackerman_set_goal_message_handler,
//			CARMEN_SUBSCRIBE_LATEST);

//	carmen_moving_objects_point_clouds_subscribe_message(NULL,
//			(carmen_handler_t) carmen_moving_objects_point_clouds_message_handler,
//			CARMEN_SUBSCRIBE_LATEST);

//	carmen_obstacle_distance_mapper_subscribe_message(NULL,
//			(carmen_handler_t) carmen_obstacle_distance_mapper_map_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_obstacle_distance_mapper_subscribe_compact_map_message(NULL,
			(carmen_handler_t) carmen_obstacle_distance_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behaviour_selector_subscribe_compact_lane_contents_message(NULL,
			(carmen_handler_t) carmen_behaviour_selector_compact_lane_contents_message_handler, CARMEN_SUBSCRIBE_LATEST);

	// carmen_voice_interface_subscribe_command_message(NULL, (carmen_handler_t) carmen_voice_interface_command_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
read_parameters_specific(int argc, char **argv)
{
	carmen_param_t optional_param_list[] = {
        {(char *) "model", 		(char *) "predictive_planner_obstacles_safe_length_distance", CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.model_predictive_planner_obstacles_safe_length_distance, 1, NULL},
        {(char *) "model", 		(char *) "predictive_planner_max_square_distance_to_lane", CARMEN_PARAM_DOUBLE, &GlobalState::max_square_distance_to_lane, 1, NULL},
		{(char *) "model", 		(char *) "predictive_planner_obstacles_safe_distance", 	CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.model_predictive_planner_obstacles_safe_distance, 1, NULL},
		{(char *) "obstacle", 	(char *) "avoider_obstacles_safe_distance", 			CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.obstacle_avoider_obstacles_safe_distance, 1, NULL},
		{(char *) "rrt",	(char *) "use_obstacle_avoider", 	CARMEN_PARAM_ONOFF,		&GlobalState::use_obstacle_avoider, 	1, NULL},
		{(char *) "rrt",	(char *) "use_mpc",					CARMEN_PARAM_ONOFF,		&GlobalState::use_mpc, 					0, NULL},
		{(char *) "rrt",	(char *) "publish_tree",			CARMEN_PARAM_ONOFF,		&GlobalState::publish_tree,				1, NULL},
		{(char *) "rrt",	(char *) "reuse_last_path",			CARMEN_PARAM_ONOFF,		&GlobalState::reuse_last_path,			1, NULL},
		{(char *) "rrt",	(char *) "obstacle_cost_distance",	CARMEN_PARAM_DOUBLE,	&GlobalState::obstacle_cost_distance,	1, NULL}
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));

	original_model_predictive_planner_obstacles_safe_distance = GlobalState::robot_config.model_predictive_planner_obstacles_safe_distance;
}


void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
		{(char *) "robot",				 (char *) "length",														CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.length,										 1, NULL},
		{(char *) "robot",				 (char *) "width",														CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.width,										 1, NULL},
		{(char *) "robot",				 (char *) "distance_between_rear_wheels",								CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_rear_wheels,				 1, NULL},
		{(char *) "robot",				 (char *) "distance_between_front_and_rear_axles",						CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_front_and_rear_axles,		 1, NULL},
		{(char *) "robot",				 (char *) "distance_between_front_car_and_front_wheels",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_front_car_and_front_wheels, 1, NULL},
		{(char *) "robot",				 (char *) "distance_between_rear_car_and_rear_wheels",					CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_rear_car_and_rear_wheels,	 1, NULL},
		{(char *) "robot",				 (char *) "max_velocity",												CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.max_v,										 1, NULL},
		{(char *) "robot",				 (char *) "max_steering_angle",											CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.max_phi,									 1, NULL},
		{(char *) "robot",				 (char *) "maximum_acceleration_forward",								CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_acceleration_forward,				 1, NULL},
		{(char *) "robot",				 (char *) "maximum_acceleration_reverse",								CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_acceleration_reverse,				 1, NULL},
		{(char *) "robot",				 (char *) "maximum_deceleration_forward",								CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_deceleration_forward,				 1, NULL},
		{(char *) "robot",				 (char *) "maximum_deceleration_reverse",								CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_deceleration_reverse,				 1, NULL},

		{(char *) "robot", 				 (char *) "desired_decelaration_forward",								CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.desired_decelaration_forward,					1, NULL},
		{(char *) "robot", 				 (char *) "desired_decelaration_reverse",								CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.desired_decelaration_reverse,					1, NULL},
		{(char *) "robot", 				 (char *) "desired_acceleration",										CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.desired_acceleration,							1, NULL},
		{(char *) "robot", 				 (char *) "desired_steering_command_rate",								CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.desired_steering_command_rate,					1, NULL},
		{(char *) "robot", 				 (char *) "understeer_coeficient",										CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.understeer_coeficient,							1, NULL},
		{(char *) "robot", 				 (char *) "maximum_steering_command_rate", 								CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_steering_command_rate, 					1, NULL},

		{(char *) "robot",				 (char *) "max_centripetal_acceleration",								CARMEN_PARAM_DOUBLE, &GlobalState::robot_max_centripetal_acceleration,						 1, NULL},
		{(char *) "robot",				 (char *) "max_velocity_reverse",										CARMEN_PARAM_DOUBLE, &GlobalState::param_max_vel_reverse,									 1, NULL},
		{(char *) "robot",				 (char *) "parking_speed_limit",										CARMEN_PARAM_DOUBLE, &GlobalState::param_parking_speed_limit,									 1, NULL},

		{(char *) "semi_trailer",		 (char *) "initial_type",												CARMEN_PARAM_INT,	 &GlobalState::semi_trailer_config.num_semi_trailers,								 0, NULL},
		{(char *) "rddf",				 (char *) "source_tracker",												CARMEN_PARAM_ONOFF,  &GlobalState::use_tracker_goal_and_lane,								 0, NULL},
		{(char *) "behavior_selector",	 (char *) "goal_source_path_planner",									CARMEN_PARAM_ONOFF,  &GlobalState::use_path_planner,										 0, NULL},
		{(char *) "behavior_selector",	 (char *) "use_truepos",												CARMEN_PARAM_ONOFF,  &GlobalState::use_truepos,												 0, NULL},
		{(char *) "behavior_selector",	 (char *) "reverse_driving",											CARMEN_PARAM_ONOFF,  &GlobalState::reverse_driving_flag,									 0, NULL},
		{(char *) "behavior_selector", 	 (char *) "distance_between_waypoints", 								CARMEN_PARAM_DOUBLE, &GlobalState::distance_between_waypoints, 1, NULL},
		{(char *) "model",				 (char *) "predictive_planner_w1_end_of_path_to_goal_distance",         CARMEN_PARAM_DOUBLE, &GlobalState::w1,														 1, NULL},
		{(char *) "model",				 (char *) "predictive_planner_w2_end_of_path_to_goal_angular_distance", CARMEN_PARAM_DOUBLE, &GlobalState::w2,														 1, NULL},
		{(char *) "model",				 (char *) "predictive_planner_w3_end_of_path_to_goal_delta_theta",      CARMEN_PARAM_DOUBLE, &GlobalState::w3,														 1, NULL},
		{(char *) "model",				 (char *) "predictive_planner_w4_path_to_lane_distance",                CARMEN_PARAM_DOUBLE, &GlobalState::w4,														 1, NULL},
		{(char *) "model",				 (char *) "predictive_planner_w5_proximity_to_obstacles",               CARMEN_PARAM_DOUBLE, &GlobalState::w5,														 1, NULL},
		{(char *) "model",				 (char *) "predictive_planner_w6_traveled_distance",                    CARMEN_PARAM_DOUBLE, &GlobalState::w6,														 1, NULL},
//		{(char *) "model",				 (char *) "predictive_planner_w7_look_ahead_error",                    	CARMEN_PARAM_DOUBLE, &GlobalState::w7,														 1, NULL},
//		{(char *) "model",				 (char *) "predictive_planner_look_ahead_horizon",                    	CARMEN_PARAM_DOUBLE, &GlobalState::look_ahead_horizon,										 1, NULL},

		{(char *) "model",				 (char *) "predictive_planner_eliminate_path_follower",					CARMEN_PARAM_ONOFF,	 &GlobalState::eliminate_path_follower,									 1, NULL},
		{(char *) "model",				 (char *) "predictive_planner_eliminate_path_follower_transition_v",    CARMEN_PARAM_DOUBLE, &GlobalState::eliminate_path_follower_transition_v,					 1, NULL},
		{(char *) "model",				 (char *) "predictive_planner_robot_velocity_delay",                    CARMEN_PARAM_DOUBLE, &GlobalState::robot_velocity_delay,									 1, NULL},
		{(char *) "model",				 (char *) "predictive_planner_robot_min_v_distance_ahead",              CARMEN_PARAM_DOUBLE, &GlobalState::robot_min_v_distance_ahead,								 1, NULL},
		{(char *) "model",				 (char *) "predictive_planner_robot_steering_delay",                    CARMEN_PARAM_DOUBLE, &GlobalState::robot_steering_delay,									 1, NULL},
		{(char *) "model",				 (char *) "predictive_planner_robot_min_s_distance_ahead",              CARMEN_PARAM_DOUBLE, &GlobalState::robot_min_s_distance_ahead,								 1, NULL},

		{(char *) "frenet_path_planner", (char *) "use_unity_simulator",										CARMEN_PARAM_ONOFF,	 &use_unity_simulator,													 0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	voice_interface_max_vel = GlobalState::param_max_vel = GlobalState::robot_config.max_v;

	read_parameters_specific(argc, argv);

	carmen_param_t param_optional_list[] =
	{
		{(char *) "commandline", (char *) "update_lookup_table", CARMEN_PARAM_ONOFF, &update_lookup_table, 0, NULL},
		{(char *) "commandline", (char *) "teacher_mode",		 CARMEN_PARAM_ONOFF, &g_teacher_mode,	   0, NULL}
	};

	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));

	if (GlobalState::semi_trailer_config.num_semi_trailers > 0)
		carmen_task_manager_read_semi_trailer_parameters(&GlobalState::semi_trailer_config, argc, argv, GlobalState::semi_trailer_config.num_semi_trailers);
}

//extern carmen_mapper_virtual_laser_message virtual_laser_message;
//#define MAX_VIRTUAL_LASER_SAMPLES 10000

int
main(int argc, char **argv)
{
//    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

	argc_global = argc;
	argv_global = argv;

    carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

	register_handlers();
	signal(SIGINT, signal_handler);
//	signal(SIGFPE, signal_handler);

//	g_trajectory_lookup_table = new TrajectoryLookupTable(update_lookup_table);
	memset((void *) &tree, 0, sizeof(Tree));

//	memset(&virtual_laser_message, 0, sizeof(carmen_mapper_virtual_laser_message));
//	virtual_laser_message.positions = (carmen_position_t *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(carmen_position_t));
//	virtual_laser_message.colors = (char *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(char));
//	virtual_laser_message.host = carmen_get_host();

	carmen_ipc_dispatch();
}

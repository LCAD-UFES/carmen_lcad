#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <carmen/collision_detection.h>

#include "robot_state.h"
#include "model/global_state.h"
#include "util.h"

#include "neural_motion_planner.h"
#include "neural_motion_planner_optimizer.h"

#include "g2o/types/slam2d/se2.h"

using namespace g2o;

int print_to_debug = 0;

extern int use_unity_simulator;


void
move_poses_foward_to_local_reference(SE2 &robot_pose, double beta __attribute__((unused)), carmen_behavior_selector_path_goals_and_annotations_message *path_goals_and_annotations_message,
		vector<carmen_robot_and_trailers_path_point_t> *lane_in_local_pose)
{
	carmen_robot_and_trailers_path_point_t local_reference_lane_point;

	int index = 0;

	for (int k = index; k < path_goals_and_annotations_message->number_of_poses; k++)
	{
		SE2 lane_in_world_reference(path_goals_and_annotations_message->poses[k].x, path_goals_and_annotations_message->poses[k].y, path_goals_and_annotations_message->poses[k].theta);
		SE2 lane_in_car_reference = robot_pose.inverse() * lane_in_world_reference;
		local_reference_lane_point = {lane_in_car_reference[0], lane_in_car_reference[1], lane_in_car_reference[2], 0, {0.0, 0.0, 0.0, 0.0, 0.0},
				path_goals_and_annotations_message->poses[k].v, path_goals_and_annotations_message->poses[k].phi, 0.0};

		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		{
			double current_beta = convert_theta1_to_beta(path_goals_and_annotations_message->poses[k].theta, path_goals_and_annotations_message->poses[k].trailer_theta[z]); // Necessário para trocar a referência do trailer_theta
			local_reference_lane_point.trailer_theta[z] = convert_beta_to_theta1(lane_in_car_reference[2], current_beta);
		}

		lane_in_local_pose->push_back(local_reference_lane_point);
	}
}


void
move_lane_to_robot_reference_system(carmen_robot_and_trailers_pose_t *localizer_pose, carmen_behavior_selector_path_goals_and_annotations_message *path_goals_and_annotations_message,
		vector<carmen_robot_and_trailers_path_point_t> *lane_in_local_pose)
{
	SE2 robot_pose(localizer_pose->x, localizer_pose->y, localizer_pose->theta);
	move_poses_foward_to_local_reference(robot_pose, localizer_pose->trailer_theta[0], path_goals_and_annotations_message, lane_in_local_pose);
}


bool
make_detailed_lane_start_at_car_pose(vector<carmen_robot_and_trailers_path_point_t> &detailed_lane,
		vector<carmen_robot_and_trailers_path_point_t> temp_detail, Pose *goal_pose)
{
	carmen_ackerman_path_point_t car_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	SE2 robot_pose(GlobalState::localizer_pose->x, GlobalState::localizer_pose->y, GlobalState::localizer_pose->theta);
	SE2 goal_in_world_reference(goal_pose->x, goal_pose->y, goal_pose->theta);
	SE2 goal_in_car_reference = robot_pose.inverse() * goal_in_world_reference;
	carmen_ackerman_path_point_t goal = {goal_in_car_reference[0], goal_in_car_reference[1], goal_in_car_reference[2], 0.0, 0.0, 0.0};

	unsigned int nearest_i_to_car = 0, nearest_i_to_goal = 0;
	double dist, nearest_distance_to_car, nearest_distance_to_goal;
	nearest_distance_to_car = nearest_distance_to_goal = DBL_MAX;
	for (unsigned int i = 0; i < temp_detail.size(); i++)
	{
		dist = DIST2D(temp_detail.at(i), car_pose);
		if (dist < nearest_distance_to_car)
		{
			nearest_distance_to_car = dist;
			nearest_i_to_car = i;
		}

		dist = DIST2D(temp_detail.at(i), goal);
		if (dist < nearest_distance_to_goal)
		{
			nearest_distance_to_goal = dist;
			nearest_i_to_goal = i;
		}
	}

	nearest_i_to_goal += 20;	// Para o tamanho da detailed lane ir um pouco alem do goal.
	if (nearest_i_to_goal > temp_detail.size())
		nearest_i_to_goal = temp_detail.size();

	for (unsigned int j = nearest_i_to_car; j < nearest_i_to_goal; j++)
		detailed_lane.push_back(temp_detail.at(j));

	return (true);
}


void
add_points_to_goal_list_interval(carmen_robot_and_trailers_path_point_t p1, carmen_robot_and_trailers_path_point_t p2,
		vector<carmen_robot_and_trailers_path_point_t> &detailed_lane)
{
	double distance = DIST2D(p1, p2);

	double distance_between_goals = 0.1;
	int num_points = floor(distance / distance_between_goals);
	if (num_points == 0)
		return;

	if (num_points > 10) // Safeguard
		num_points = 10;

	double delta_x = (p2.x - p1.x) / (double) num_points;
	double delta_y = (p2.y - p1.y) / (double) num_points;
	double delta_theta = carmen_normalize_theta(p2.theta - p1.theta) / (double) num_points;
	double delta_beta = carmen_normalize_theta(p2.trailer_theta[0] - p1.trailer_theta[0]) / (double) num_points;

	carmen_robot_and_trailers_path_point_t new_point = {p1.x, p1.y, p1.theta, p1.num_trailers, {0.0}, p1.v, p1.phi, 0.0}; // necessario para capturar v e phi

	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		new_point.trailer_theta[z] = p1.trailer_theta[z];

	for (int i = 0; i < num_points; i++)
	{
		new_point.x = p1.x + (double) i * delta_x;
		new_point.y = p1.y + (double) i * delta_y;
		new_point.theta = carmen_normalize_theta(p1.theta + (double) i * delta_theta);
		new_point.trailer_theta[0] = carmen_normalize_theta(p1.trailer_theta[0] + (double) i * delta_beta);
		for (size_t z = 1; z < MAX_NUM_TRAILERS; z++)
		{
			double current_delta_trailer_theta = carmen_normalize_theta(p2.trailer_theta[z] - p1.trailer_theta[z]) / (double) num_points;
			new_point.trailer_theta[z] = carmen_normalize_theta(p1.trailer_theta[z] + (double) i * current_delta_trailer_theta);
		}

		detailed_lane.push_back(new_point);
	}
}


bool
build_detailed_path_lane(vector<carmen_robot_and_trailers_path_point_t> *lane_in_local_pose, vector<carmen_robot_and_trailers_path_point_t> &detailed_lane)
{
	if (lane_in_local_pose->size() > 1 && lane_in_local_pose->size() < 500)
	{
		for (unsigned int i = 0; i < (lane_in_local_pose->size() - 1); i++)
		{
			add_points_to_goal_list_interval(lane_in_local_pose->at(i), lane_in_local_pose->at(i+1), detailed_lane);
			if (detailed_lane.size() > 1000)
			{
				detailed_lane.clear();
				return false;
			}
		}
		detailed_lane.push_back(lane_in_local_pose->back());
	}
	else
	{
		if (print_to_debug)
			printf(KGRN "+++++++++++++ ERRO MENSAGEM DA LANE POSES !!!!\n" RESET);
		detailed_lane.clear();
		return (false);
	}
	return (true);
}


bool
build_detailed_rddf_lane(Pose *goal_pose, vector<carmen_robot_and_trailers_path_point_t> *lane_in_local_pose,
		vector<carmen_robot_and_trailers_path_point_t> &detailed_lane)
{
	bool goal_in_lane = false;
	if (lane_in_local_pose->size() > 1)
	{
		vector<carmen_robot_and_trailers_path_point_t> temp_detail;
		for (unsigned int i = 0; i < (lane_in_local_pose->size() - 1); i++)
			add_points_to_goal_list_interval(lane_in_local_pose->at(i), lane_in_local_pose->at(i+1), temp_detail);
		temp_detail.push_back(lane_in_local_pose->back());

		goal_in_lane = make_detailed_lane_start_at_car_pose(detailed_lane, temp_detail, goal_pose);

	}
	else
	{
		if (print_to_debug)
			printf(KGRN "+++++++++++++ ERRO MENSAGEM DA LANE POSES !!!!\n" RESET);
		return (false);
	}

	return (goal_in_lane);
}


bool
path_has_collision_or_phi_exceeded(vector<carmen_robot_and_trailers_path_point_t> &path)
{
	double circle_radius = GlobalState::robot_config.obstacle_avoider_obstacles_safe_distance;
	carmen_robot_and_trailers_pose_t localizer = {GlobalState::localizer_pose->x, GlobalState::localizer_pose->y,
			GlobalState::localizer_pose->theta, GlobalState::localizer_pose->num_trailers, {0.0}};

	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		localizer.trailer_theta[z] = GlobalState::localizer_pose->trailer_theta[z];


	double max_circle_invasion;
	for (int j = 0; j < 1; j++)
	{
		max_circle_invasion = 0.0;
		for (unsigned int i = 0; i < path.size(); i += 1)
		{
			if ((path[i].phi > GlobalState::robot_config.max_phi) ||
				(path[i].phi < -GlobalState::robot_config.max_phi))
			{
				static double last_time = 0.0;
				if ((carmen_get_time() - last_time) > 0.1)
				{
					printf("---------- PHI EXCEEDED THE MAX_PHI!!!!\n");
					last_time = carmen_get_time();
				}
			}

			carmen_robot_and_trailers_pose_t point_to_check = {path[i].x, path[i].y, path[i].theta, path[i].num_trailers, {0.0}};

			for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
				point_to_check.trailer_theta[z] = path[i].trailer_theta[z];

			if (GlobalState::distance_map != NULL)
			{
				double circle_invasion = sqrt(carmen_obstacle_avoider_proximity_to_obstacles(&localizer,
						point_to_check, GlobalState::distance_map, circle_radius));

				if (circle_invasion > max_circle_invasion)
					max_circle_invasion = circle_invasion;
			}
		}


		if ((GlobalState::distance_map != NULL) && (max_circle_invasion > 0.0))// GlobalState::distance_map->config.resolution / 2.0))
			path.erase(path.begin() + (int) (0.7 * (double) path.size()), path.begin() + path.size());
		else
			return (false);
	}

	if ((GlobalState::distance_map != NULL) && (max_circle_invasion > 0.0))// GlobalState::distance_map->config.resolution / 2.0))
	{
		static double last_time = 0.0;
		if ((carmen_get_time() - last_time) > 0.1)
		{
//			printf("---------- PATH HIT OBSTACLE!!!! -> %lf\n", carmen_get_time());
			last_time = carmen_get_time();
		}
		return (true);
	}
	else
		return (false);
}


bool
get_path_from_optimized_tcp(vector<carmen_robot_and_trailers_path_point_t> &path,
		vector<carmen_robot_and_trailers_path_point_t> &path_local,
		TrajectoryControlParameters otcp,
		TrajectoryDimensions td,
		carmen_robot_and_trailers_pose_t *localizer_pose)
{
	if (GlobalState::use_mpc)
		path = simulate_car_from_parameters(td, otcp, td.v_i, 0.025);
	else if (use_unity_simulator)
		path = simulate_car_from_parameters(td, otcp, td.v_i, 0.02);
	else if (GlobalState::eliminate_path_follower)
		path = simulate_car_from_parameters(td, otcp, td.v_i, 0.02);
	else
		path = simulate_car_from_parameters(td, otcp, td.v_i);

	path_local = path;

	if (path_has_loop(td.dist, otcp.sf))
	{
		printf(KRED "+++++++++++++ Path has loop...\n" RESET);
		return (false);
	}

	if (path_has_collision_or_phi_exceeded(path))
		GlobalState::path_has_collision_or_phi_exceeded = true;
	else
		GlobalState::path_has_collision_or_phi_exceeded = false;

	move_path_to_current_robot_pose(path, localizer_pose);
	return (true);
}


bool
set_reverse_planning_global_state(double target_v, double v_i, TrajectoryControlParameters &previous_good_tcp)
{
	if (!GlobalState::reverse_driving_flag && ((target_v < 0.0) || (v_i < 0.0)))
	{
		printf(KGRN "+++++++++++++ REVERSE DRIVING NAO ESTA ATIVO NOS PARAMETROS - LANE DESCARTADA!!!!\n" RESET);
		return (false);
	}

	if (((target_v < 0.0) || (v_i < -0.005)) && GlobalState::reverse_driving_flag)
	{
		if (GlobalState::reverse_planning == 0)
			previous_good_tcp.valid = false;

		GlobalState::reverse_planning = 1;
	}
	else
	{
		if (GlobalState::reverse_planning == 1)
			previous_good_tcp.valid = false;

		GlobalState::reverse_planning = 0;
	}

	return (true);
}


TrajectoryDimensions
get_trajectory_dimensions_from_robot_state(carmen_robot_and_trailers_pose_t *localizer_pose, Command last_odometry,	Pose *goal_pose)
{
	TrajectoryDimensions td;
	td.dist = sqrt((goal_pose->x - localizer_pose->x) * (goal_pose->x - localizer_pose->x) +
			(goal_pose->y - localizer_pose->y) * (goal_pose->y - localizer_pose->y));
	td.theta = carmen_normalize_theta(atan2(goal_pose->y - localizer_pose->y, goal_pose->x - localizer_pose->x) - localizer_pose->theta);
	td.d_yaw = carmen_normalize_theta(goal_pose->theta - localizer_pose->theta);
	td.phi_i = last_odometry.phi;
	td.v_i = last_odometry.v;

	if (GlobalState::semi_trailer_config.num_semi_trailers == 0)
	{
		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			td.trailer_theta_i[z] = 0.0;
	}
	else
	{
		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			td.trailer_theta_i[z] = carmen_normalize_theta(-convert_theta1_to_beta(localizer_pose->theta, localizer_pose->trailer_theta[z])); 
	}

	SE2 robot_pose(localizer_pose->x, localizer_pose->y,localizer_pose->theta);
	SE2 goal_in_world_reference(goal_pose->x, goal_pose->y, goal_pose->theta);
	SE2 goal_in_car_reference = robot_pose.inverse() * goal_in_world_reference;
	td.goal_pose.x = goal_in_car_reference[0];
	td.goal_pose.y = goal_in_car_reference[1];
	td.goal_pose.theta = goal_in_car_reference[2];
	return (td);
}


vector<vector<carmen_robot_and_trailers_path_point_t> >
compute_path_to_goal(carmen_robot_and_trailers_pose_t *localizer_pose, Pose *goal_pose, Command last_odometry, double target_v,
		carmen_behavior_selector_path_goals_and_annotations_message *path_goals_and_annotations_message)
{
	vector<vector<carmen_robot_and_trailers_path_point_t>> paths;
	vector<carmen_robot_and_trailers_path_point_t> lane_in_local_pose, detailed_lane;
	static TrajectoryControlParameters previous_good_tcp = {};
	static bool first_time = true;
	bool goal_in_lane = false;

	if (first_time || !GlobalState::following_path)
	{
		previous_good_tcp.valid = false;
		first_time = false;
	}

	paths.resize(1);
	if (!set_reverse_planning_global_state(target_v, last_odometry.v, previous_good_tcp))
	{
		paths.clear();
		return (paths);
	}

	move_lane_to_robot_reference_system(localizer_pose, path_goals_and_annotations_message, &lane_in_local_pose);

	if (GlobalState::use_path_planner || GlobalState::use_tracker_goal_and_lane)
	{
		build_detailed_path_lane(&lane_in_local_pose, detailed_lane);
	}
	else
	{
		goal_in_lane = build_detailed_rddf_lane(goal_pose, &lane_in_local_pose, detailed_lane);
		if (!goal_in_lane)
			detailed_lane.clear();
	}

	bool use_lane = true;
	TrajectoryDimensions td = get_trajectory_dimensions_from_robot_state(localizer_pose, last_odometry, goal_pose);
	TrajectoryControlParameters otcp = get_complete_optimized_trajectory_control_parameters(previous_good_tcp, td, target_v, detailed_lane, use_lane);

	if (otcp.valid)
	{
		vector<carmen_robot_and_trailers_path_point_t> path;
		vector<carmen_robot_and_trailers_path_point_t> path_local;

		if (!get_path_from_optimized_tcp(path, path_local, otcp, td, localizer_pose))
		{
			paths.clear();
			return (paths);
		}

		paths[0] = path;
		fflush(stdout);
	}

	return (paths);
}

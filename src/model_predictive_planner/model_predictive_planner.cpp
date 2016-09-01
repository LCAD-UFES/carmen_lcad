/*
 * model_predictive_planner.cpp
 *
 *  Created on: Jun 22, 2016
 *      Author: alberto
 */
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>

#include <carmen/collision_detection.h>

#include "model/robot_state.h"
#include "model/global_state.h"
#include "util.h"

#include "model_predictive_planner.h"

#include "g2o/types/slam2d/se2.h"

#define SYSTEM_DELAY 0.3

using namespace g2o;

TrajectoryLookupTable::CarLatencyBuffer g_car_latency_buffer_mp;


TrajectoryLookupTable::TrajectoryDimensions
get_trajectory_dimensions_from_robot_state(Pose *localizer_pose, Command last_odometry,	Pose *goal_pose)
{
	TrajectoryLookupTable::TrajectoryDimensions td;

	td.dist = sqrt((goal_pose->x - localizer_pose->x) * (goal_pose->x - localizer_pose->x) +
			(goal_pose->y - localizer_pose->y) * (goal_pose->y - localizer_pose->y));
	td.theta = carmen_normalize_theta(atan2(goal_pose->y - localizer_pose->y, goal_pose->x - localizer_pose->x) - localizer_pose->theta);
	td.d_yaw = carmen_normalize_theta(goal_pose->theta - localizer_pose->theta);
	td.phi_i = last_odometry.phi;
	td.v_i = last_odometry.v;

	return (td);
}


bool
move_lane_to_robot_reference_system(Pose *localizer_pose, carmen_rddf_road_profile_message *goal_list_message,
		Pose *goal_pose, vector<carmen_ackerman_path_point_t> *lane_in_local_pose)
{
	double last_dist = DBL_MAX;
	double dist = 0.0;

	SE2 robot_pose(localizer_pose->x, localizer_pose->y, localizer_pose->theta);
	SE2 goal_in_world_reference(goal_pose->x, goal_pose->y, goal_pose->theta);
	SE2 goal_in_car_reference = robot_pose.inverse() * goal_in_world_reference;
	double goal_x = goal_in_car_reference[0];
	double goal_y = goal_in_car_reference[1];
	//	printf("Goal x: %lf y: %lf \n", goal_x, goal_y);

	if ((goal_list_message->number_of_poses < 2) && (goal_list_message->number_of_poses_back < 2))
		return false;

	vector<carmen_ackerman_path_point_t> poses_back;
	carmen_ackerman_path_point_t local_reference_lane_point;

	//Get the back poses

	for (int i = 0; i < goal_list_message->number_of_poses_back; i++)
	{
		SE2 lane_back_in_world_reference(goal_list_message->poses_back[i].x, goal_list_message->poses_back[i].y, goal_list_message->poses_back[i].theta);
		SE2 lane_back_in_car_reference = robot_pose.inverse() * lane_back_in_world_reference;

		local_reference_lane_point = {lane_back_in_car_reference[0], lane_back_in_car_reference[1], lane_back_in_car_reference[2],
				goal_list_message->poses_back[i].v, goal_list_message->poses_back[i].phi, 0.0};

		poses_back.push_back(local_reference_lane_point);

		if (local_reference_lane_point.x <= 0)
		{
			for (int j = (poses_back.size() - 1); j >= 0 ; j--)
				lane_in_local_pose->push_back(poses_back.at(j));
			break;
		}
	}

	int index = 0;
	if (goal_list_message->poses[0].x == goal_list_message->poses[1].x && goal_list_message->poses[0].y == goal_list_message->poses[1].y)
		index = 1;

	for (int k = index; k < goal_list_message->number_of_poses; k++)
	{

		SE2 lane_in_world_reference(goal_list_message->poses[k].x, goal_list_message->poses[k].y, goal_list_message->poses[k].theta);
		SE2 lane_in_car_reference = robot_pose.inverse() * lane_in_world_reference;


		local_reference_lane_point = {lane_in_car_reference[0], lane_in_car_reference[1], lane_in_car_reference[2],
				goal_list_message->poses[k].v, goal_list_message->poses[k].phi, 0.0};

		lane_in_local_pose->push_back(local_reference_lane_point);

		if (local_reference_lane_point.x == goal_x && local_reference_lane_point.y == goal_y)
			return true;

		dist = sqrt((carmen_square(local_reference_lane_point.x - goal_x) + carmen_square(local_reference_lane_point.y - goal_y)));
		if (last_dist < dist)
			return false;
		last_dist = dist;
	}
	return false;
}


void
add_points_to_goal_list_interval(carmen_ackerman_path_point_t p1, carmen_ackerman_path_point_t p2, vector<carmen_ackerman_path_point_t> &detailed_lane)
{

	//double theta;
	double delta_x, delta_y, delta_theta, distance;
	int i;

	distance = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));

	double distance_between_goals = 0.1;
	int num_points = distance / distance_between_goals;

	//  NOTA IMPORTANTISSIMA: ESTUDAR POR QUE O CODIGO COMENTADO NAO DA O MESMO
	//  RESULTADO QUE O CODIGO ABAIXO!!!!
	//	theta = atan2(p2.y - p1.y, p2.x - p1.x);
	//	delta_x = (distance * cos(theta)) / (double) num_points;
	//	delta_y = (distance * sin(theta)) / (double) num_points;

	delta_x = (p2.x - p1.x) / num_points;
	delta_y = (p2.y - p1.y) / num_points;
	delta_theta = carmen_normalize_theta((p2.theta - p1.theta)) / num_points;

	carmen_ackerman_path_point_t new_point = {p1.x, p1.y, p1.theta, p1.v, p1.phi, 0.0};

	for (i = 0; i < num_points; i++)
	{
		new_point.x = p1.x + i * delta_x;
		new_point.y = p1.y + i * delta_y;
		new_point.theta = carmen_normalize_theta(p1.theta + i * delta_theta);

		detailed_lane.push_back(new_point);
	}
}


void
copy_starting_nearest_point_of_zero(vector<carmen_ackerman_path_point_t> &detailed_lane, vector<carmen_ackerman_path_point_t> &temp_detail)
{
	detailed_lane.clear();

	//mantendo primeiro ponto mais proximo de 0
	for (unsigned int i = 1; i < temp_detail.size(); i++)
	{
		//		printf("Temp x: %lf y: %lf \n", temp_detail.at(i).x, temp_detail.at(i).y);
		if (temp_detail.at(i).x > 0.0)
		{
			double distance1 = sqrt((carmen_square(temp_detail.at(i-1).x - 0.0) + carmen_square(temp_detail.at(i-1).y - 0.0)));
			double distance2 = sqrt((carmen_square(temp_detail.at(i).x - 0.0) + carmen_square(temp_detail.at(i).y - 0.0)));
			if ((distance1 < distance2))
				i--;
			// slice
			int k = 0;
			for (unsigned int j = i; j < temp_detail.size(); j++ , k++)
				detailed_lane.push_back(temp_detail.at(j));
			break;
		}
	}
}


bool
build_detailed_lane(vector<carmen_ackerman_path_point_t> *lane_in_local_pose, vector<carmen_ackerman_path_point_t> &detailed_lane)
{
	if (lane_in_local_pose->size() > 2)
	{
		vector<carmen_ackerman_path_point_t> temp_detail;
		for (unsigned int i = 0; i < (lane_in_local_pose->size() - 1); i++)
			add_points_to_goal_list_interval(lane_in_local_pose->at(i), lane_in_local_pose->at(i+1), temp_detail);

		//add last point
		temp_detail.push_back(lane_in_local_pose->back());
		//mantendo primeiro ponto mais proximo de 0
		copy_starting_nearest_point_of_zero(detailed_lane, temp_detail);
	}
	else
	{
		printf(KGRN "+++++++++++++ ERRO MENSAGEM DA LANE POSES !!!!\n" RESET);
		detailed_lane.clear();
		return (false);
	}
	return (true);
}


int
find_pose_in_path(double &time_displacement, Pose *localizer_pose, vector<carmen_ackerman_path_point_t> path)
{
	time_displacement = 0.0;
	carmen_ackerman_path_point_t p;
	unsigned int i;

	for (i = 0; i < (path.size() - 1); i++)
		if (Util::between(p, localizer_pose, path[i], path[i+1]))
			break;

	time_displacement = p.time;

	return (i);
}


double
compute_delay(vector<carmen_ackerman_path_point_t> path, int path_index,
		double localizer_pose_timestamp, double path_timestamp, double time_displacement)
{
	double path_time_up_to_localizer_pose = path_timestamp;
	for (int i = 0; i < path_index; i++)
		path_time_up_to_localizer_pose += path[i].time;
	path_time_up_to_localizer_pose += time_displacement;

	double delay = localizer_pose_timestamp - path_time_up_to_localizer_pose;

	return (delay);
}


void
apply_system_delay(vector<carmen_ackerman_path_point_t> &path)
{
	double delay = 0.0;
	unsigned int index;

	if (path.size() < 1)
		return;

	for (index = 0; (delay < SYSTEM_DELAY) && (index < path.size()); index++)
		delay += path[index].time;
	for (unsigned int j = 0; j < (index - 1); j++)
		path.erase(path.begin());

	if (index < path.size())
	{
		path[0].time -= delay - SYSTEM_DELAY;
		// std::cout << "$$$$$$$$$$$$$$ index = " << index << "  time discount = " << delay - SYSTEM_DELAY << std::endl;
	}
}


void
filter_path(vector<carmen_ackerman_path_point_t> &path)
{
	unsigned int i;

	if (path.size() < 1)
		return;

	for (i = 0; i < path.size(); i += 2)
		if ((i + 1) < path.size())
			path[i].time += path[i + 1].time;
	for (i = 1; i < path.size(); i += 2)
		path.erase(path.begin() + i);

	for (i = 0; i < path.size(); i += 2)
		if ((i + 1) < path.size())
			path[i].time += path[i + 1].time;
	for (i = 1; i < path.size(); i += 2)
		path.erase(path.begin() + i);

	if (GlobalState::ford_escape_online)
	{
		for (i = 0; i < path.size(); i += 2)
			if ((i + 1) < path.size())
				path[i].time += path[i + 1].time;
		for (i = 1; i < path.size(); i += 2)
			path.erase(path.begin() + i);
	}
}


void
apply_system_latencies(vector<carmen_ackerman_path_point_t> &path)
{
	unsigned int i, j;

	for (i = 0; i < path.size(); i++)
	{
		j = i;
		for (double lat = 0.0; lat < 0.3; j++)
		{
			if (j >= path.size())
				break;
			lat += path[j].time;
		}
		if (j >= path.size())
			break;
		path[i].phi = path[j].phi;
	}
	//    while (i < path.size())
		//    	path.pop_back();

	for (i = 0; i < path.size(); i++)
	{
		j = i;
		for (double lat = 0.0; lat < 0.2; j++)
		{
			if (j >= path.size())
				break;
			lat += path[j].time;
		}
		if (j >= path.size())
			break;
		path[i].v = path[j].v;
	}
	while (i < path.size())
		path.pop_back();
}


void
write_tdd_to_file(FILE *problems, TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd, string label)
{
	fprintf(problems, "tdd: %s dist: %d, theta: %d, phi_i: %d, v_i: %d, d_yaw: %d\n", label.c_str(),
			tdd.dist, tdd.theta, tdd.phi_i, tdd.v_i, tdd.d_yaw);

	TrajectoryLookupTable::TrajectoryControlParameters tcp;
	TrajectoryLookupTable::TrajectoryDimensions td = convert_to_trajectory_dimensions(tdd, tcp);
	fprintf(problems, "td: %s dist: %lf, theta: %lf, phi_i: %lf, v_i: %lf, d_yaw: %lf\n", label.c_str(),
			td.dist, td.theta, td.phi_i, td.v_i, td.d_yaw);

	fflush(problems);
}


bool
path_has_collision_or_phi_exceeded(vector<carmen_ackerman_path_point_t> path)
{
	double proximity_to_obstacles_for_path = 0.0;
	double circle_radius = (GlobalState::robot_config.width + 0.4) / 2.0; // metade da largura do carro + um espacco de guarda

	for (unsigned int i = 0; i < path.size(); i += 1)
	{
		if ((path[i].phi > GlobalState::robot_config.max_phi) ||
				(path[i].phi < -GlobalState::robot_config.max_phi))
		{
			printf("---------- PHI EXCEEDED THE MAX_PHI!!!!\n");
			return (true);
		}

		proximity_to_obstacles_for_path += compute_distance_to_closest_obstacles(path[i], circle_radius,
				&GlobalState::robot_config, GlobalState::localizer_pose, GlobalState::distance_map);
	}

	if (proximity_to_obstacles_for_path > 0.0)
	{
		printf("---------- PATH HIT OBSTACLE!!!!\n");
		return (true);
	}
	else
		return (false);
}


bool
path_has_collision_old(vector<carmen_ackerman_path_point_t> path)
{
	carmen_point_t pose;

	for (unsigned int j = 0; j < path.size(); j++)
	{
		pose.x = path[j].x;
		pose.y = path[j].y;
		pose.theta = path[j].theta;
		if (obstacle_avoider_pose_hit_obstacle(pose, &GlobalState::cost_map, &GlobalState::robot_config))
		{
			printf("---------- PATH HIT OBSTACLE!!!!\n");
			return (true);
		}
	}
	return (false);
}


void
put_shorter_path_in_front(vector<vector<carmen_ackerman_path_point_t> > &paths, int shorter_path)
{
	if ((paths.size() > 1) && (paths[0].size() == 0))
	{
		vector<carmen_ackerman_path_point_t> shoter_path;
		shoter_path = paths[shorter_path];
		paths.erase(paths.begin() + shorter_path);
		paths.insert(paths.begin(), shoter_path);
	}

	unsigned int size, i, j;
	size = paths.size();
	for (i = j = 0; i < size; i++)
	{
		if (paths[j].size() == 0)
		{
			paths.erase(paths.begin() + j);
		}
		else
			j++;
	}
}


void
add_plan_segment_to_car_latency_buffer(vector<carmen_ackerman_path_point_t> path)
{
	double t = 0.0;

	unsigned int j;
	for (int i = j = 0; i < (int) (MAX_PLANNING_TIME / LATENCY_CICLE_TIME); i++)
	{
		g_car_latency_buffer_mp.previous_v[i + V_LATENCY_BUFFER_SIZE] = path[j].v;
		g_car_latency_buffer_mp.previous_phi[i + PHI_LATENCY_BUFFER_SIZE] = path[j].phi;
		if ((double) i * LATENCY_CICLE_TIME > t)
		{
			t += path[j].time;
			j++;
			if (j >= path.size())
				break;
		}
	}
}


void
add_odometry_to_car_latency_buffer(Command c)
{
	if (g_car_latency_buffer_mp.timestamp != 0.0)
	{
		int last_delay = (GlobalState::localizer_pose_timestamp - g_car_latency_buffer_mp.timestamp) / LATENCY_CICLE_TIME;
		for (int i = 0; i < V_LATENCY_BUFFER_TOTAL_SIZE - last_delay; i++)
			g_car_latency_buffer_mp.previous_v[i + last_delay] = g_car_latency_buffer_mp.previous_v[i];
		for (int i = 0; i < last_delay; i++)
			g_car_latency_buffer_mp.previous_v[i] = c.v;

		for (int i = 0; i < PHI_LATENCY_BUFFER_TOTAL_SIZE - last_delay; i++)
			g_car_latency_buffer_mp.previous_phi[i + last_delay] = g_car_latency_buffer_mp.previous_phi[i];
		for (int i = 0; i < last_delay; i++)
			g_car_latency_buffer_mp.previous_phi[i] = c.phi;
	}

	g_car_latency_buffer_mp.timestamp = GlobalState::localizer_pose_timestamp;
}


TrajectoryLookupTable::TrajectoryControlParameters
get_shorter_path(int &shorter_path, int num_paths, vector<vector<carmen_ackerman_path_point_t> > paths,
		const vector<TrajectoryLookupTable::TrajectoryControlParameters> otcps)
{
	double shorter_path_size = 1000.0;
	TrajectoryLookupTable::TrajectoryControlParameters best_otcp;

	for (int i = 0; i < num_paths; i++)
	{
		// que garantia temos que otcps[i] é valido?????
		// paths[i] contém vários paths provienientes do lixo da memória
		if ((paths[i].size() > 0) && (otcps[i].sf < shorter_path_size) && otcps[i].valid)
		{
			shorter_path_size = otcps[i].sf;
			shorter_path = i;
			best_otcp = otcps[i];
		}
	}
	return best_otcp;
}


bool
get_tcp_from_td(TrajectoryLookupTable::TrajectoryControlParameters &tcp,
		TrajectoryLookupTable::TrajectoryControlParameters previous_good_tcp,
		TrajectoryLookupTable::TrajectoryDimensions td)
{
	if (!previous_good_tcp.valid)
	{
		TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd = get_discrete_dimensions(td);
		if (!has_valid_discretization(tdd))
		{
			printf("Invalid discretization!!!!\n");
			return (false);
		}

		tcp = search_lookup_table(tdd);
		if (!tcp.valid)
		{
			printf(KMAG "@@@@@@@@@@@ Could not find a valid entry in the table!!!!\n\033[0m");
			return (false);
		}
	}
	else
		tcp = previous_good_tcp;

	return (true);
}


bool
get_path_from_optimized_tcp(vector<carmen_ackerman_path_point_t> &path,
		TrajectoryLookupTable::TrajectoryControlParameters otcp,
		TrajectoryLookupTable::TrajectoryDimensions td,
		Pose *localizer_pose)
{
	path = simulate_car_from_parameters(td, otcp, td.v_i, td.phi_i, g_car_latency_buffer_mp, false);
	if (path_has_loop(td.dist, otcp.sf))
	{
		printf(KRED "+++++++++++++ Path has loop...\n" RESET);
		return (false);
	}

	if (path_has_collision_or_phi_exceeded(path))
		return (false);

	move_path_to_current_robot_pose(path, localizer_pose);
	//	apply_system_delay(path);
	//	apply_system_latencies(path);

	filter_path(path);

	return (true);
}


bool
goal_pose_vector_too_different(Pose goal_pose, Pose localizer_pose)
{
	static Pose g_last_goal_pose;
	bool too_different = false;

	if (g_last_goal_pose.x == 0.0 && g_last_goal_pose.y == 0.0)
	{
		g_last_goal_pose = goal_pose;
		return (true);
	}

	double distance_between_goals = g_last_goal_pose.distance(goal_pose);
	double distance_to_goal = localizer_pose.distance(goal_pose);
	if ((distance_between_goals / distance_to_goal) > 0.2) // 20%
		too_different = true;

	double angle_diff = g_last_goal_pose.get_theta_diff(goal_pose);
	angle_diff = rad2deg(angle_diff);
	if (angle_diff > 10.0)
		too_different = true;

	g_last_goal_pose = goal_pose;
	if (too_different)
		printf("TOO DIFFERENT!!!\n");

	return (too_different);
}


void
compute_paths(const vector<Command> &lastOdometryVector, vector<Pose> &goalPoseVector, double target_v,
		Pose *localizer_pose, vector<vector<carmen_ackerman_path_point_t> > &paths,
		carmen_rddf_road_profile_message *goal_list_message)
{
	vector<carmen_ackerman_path_point_t> lane_in_local_pose, detailed_lane;
	static TrajectoryLookupTable::TrajectoryControlParameters previous_good_tcp;
	vector<TrajectoryLookupTable::TrajectoryControlParameters> otcps;
	static bool first_time = true;
	static double last_timestamp = 0.0;

	if (first_time)
	{
		previous_good_tcp.valid = false;
		first_time = false;
		last_timestamp = carmen_get_time();
	}

	bool goal_in_lane = false;
	goal_in_lane = move_lane_to_robot_reference_system(localizer_pose, goal_list_message, &goalPoseVector[0], &lane_in_local_pose);

	if (!goal_in_lane)
		lane_in_local_pose.clear();

	build_detailed_lane(&lane_in_local_pose, detailed_lane);

	otcps.resize(paths.size());
	bool has_valid_path = false;
	//	#pragma omp parallel num_threads(5)
	//	{
	for (unsigned int i = 0; i < lastOdometryVector.size(); i++)
	{
		//		#pragma omp for
		for (unsigned int j = 0; j < goalPoseVector.size(); j++)
		{
			bool use_lane;
			if (j == 0)
			{
				use_lane = true;
				if (goal_pose_vector_too_different(goalPoseVector.at(0), *localizer_pose))
				{
					previous_good_tcp.valid = false;
				}
			}
			else
				use_lane = false;

			TrajectoryLookupTable::TrajectoryDimensions td = get_trajectory_dimensions_from_robot_state(localizer_pose, lastOdometryVector[i], &goalPoseVector[j]);
			TrajectoryLookupTable::TrajectoryControlParameters tcp;
			// previous_good_tcp.valid = false;
			if (!get_tcp_from_td(tcp, previous_good_tcp, td))
				continue;

			TrajectoryLookupTable::TrajectoryControlParameters otcp;
			otcp = get_complete_optimized_trajectory_control_parameters(tcp, td, target_v, detailed_lane,
					use_lane, previous_good_tcp.valid);
			//otcp = get_optimized_trajectory_control_parameters(tcp, td, target_v, &lane_in_local_pose);

			if (otcp.valid)
			{
				vector<carmen_ackerman_path_point_t> path;
				if (!get_path_from_optimized_tcp(path, otcp, td, localizer_pose))
					continue;

				paths[j + i * lastOdometryVector.size()] = path;
				otcps[j + i * lastOdometryVector.size()] = otcp;

				has_valid_path = true;
				break;
			}
			else
			{
				otcps[j + i * lastOdometryVector.size()] = otcp;
				printf(KYEL "+++++++++++++ Could NOT optimize !!!!\n" RESET);
			}
		}

		if (has_valid_path) // If could find a good path, break. Otherwise, try swerve
			break;
	}
	//	}

	int shorter_path = -1;
	TrajectoryLookupTable::TrajectoryControlParameters best_otcp;
	best_otcp =	get_shorter_path(shorter_path, (lastOdometryVector.size() * goalPoseVector.size()), paths, otcps);

	if (shorter_path >= 0)
		put_shorter_path_in_front(paths, shorter_path);
	else
		paths.clear();

	if (has_valid_path)
	{
		previous_good_tcp = best_otcp;
		last_timestamp = carmen_get_time();

		// Mostra a detailed_lane como um plano nas interfaces
//		move_path_to_current_robot_pose(detailed_lane, localizer_pose);
//		filter_path(detailed_lane);
//		filter_path(detailed_lane);
//		paths.push_back(detailed_lane);
	}
	else if ((carmen_get_time() - last_timestamp) > 0.5)
		previous_good_tcp.valid = false;

	//		add_plan_segment_to_car_latency_buffer(path[0]);
}


vector<vector<carmen_ackerman_path_point_t>>
ModelPredictive::compute_path_to_goal(Pose *localizer_pose, Pose *goal_pose, Command last_odometry,
		double target_v, carmen_rddf_road_profile_message *goal_list_message)
{
	vector<Command> lastOdometryVector;
	vector<Pose> goalPoseVector;

	double i_time = carmen_get_time();

	vector<int> magicSignals = {0, 1, -1, 2, -2, 3, -3,  4, -4,  5, -5};
	// @@@ Tranformar os dois loops abaixo em uma funcao -> compute_alternative_path_options()
	for (int i = 0; i < 1; i++)
	{
		Command newOdometry = last_odometry;
		newOdometry.phi +=  0.15 * (double) magicSignals[i]; //(0.5 / (newOdometry.v + 1))
		lastOdometryVector.push_back(newOdometry);
	}

	for (int i = 0; i < 1; i++)
	{
		//printf("Goal x: %lf Goal y: %lf \n",goal_pose->x, goal_pose->y);
		Pose newPose = *goal_pose;
		newPose.x += 0.3 * (double) magicSignals[i] * cos(carmen_normalize_theta((goal_pose->theta) - carmen_degrees_to_radians(90.0)));
		newPose.y += 0.3 * (double) magicSignals[i] * sin(carmen_normalize_theta((goal_pose->theta) - carmen_degrees_to_radians(90.0)));
		goalPoseVector.push_back(newPose);
	}

	vector<vector<carmen_ackerman_path_point_t>> paths;
	paths.resize(lastOdometryVector.size() * goalPoseVector.size());

	compute_paths(lastOdometryVector, goalPoseVector, target_v, localizer_pose, paths, goal_list_message);

	printf("%ld plano(s), tp = %lf\n", paths.size(), carmen_get_time() - i_time);
	fflush(stdout);

	return (paths);
}

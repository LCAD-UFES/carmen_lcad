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

#include "robot_state.h"
#include "model/global_state.h"
#include "util.h"

#include "model_predictive_planner.h"
#include "model_predictive_planner_optimizer.h"

#include "g2o/types/slam2d/se2.h"

using namespace g2o;

int print_to_debug = 0;
int plot_to_debug = 0;

extern int use_unity_simulator;

//#define EXPERIMENT_DATA
//#define PLOT_COLLISION


void
plot_path_and_colisions_points(vector<carmen_ackerman_path_point_t> &robot_path, vector<carmen_ackerman_path_point_t> &collision_points)
{
//	plot data Table - Last TCP - Optmizer tcp - Lane
	//Plot Optmizer step tcp and lane?

	#define DELTA_T (1.0 / 40.0)

//	#define PAST_SIZE 300
	static bool first_time = true;
	static FILE *gnuplot_pipeMP;


	if (first_time)
	{
		first_time = false;

		gnuplot_pipeMP = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipeMP, "set xrange [0:70]\n");
		fprintf(gnuplot_pipeMP, "set yrange [-10:10]\n");
//		fprintf(gnuplot_pipe, "set y2range [-0.55:0.55]\n");
		fprintf(gnuplot_pipeMP, "set xlabel 'senconds'\n");
		fprintf(gnuplot_pipeMP, "set ylabel 'effort'\n");
//		fprintf(gnuplot_pipe, "set y2label 'phi (radians)'\n");
//		fprintf(gnuplot_pipe, "set ytics nomirror\n");
//		fprintf(gnuplot_pipe, "set y2tics\n");
		fprintf(gnuplot_pipeMP, "set tics out\n");
	}

	FILE *gnuplot_data_file = fopen("gnuplot_data_path.txt", "w");
	FILE *gnuplot_data_lane = fopen("gnuplot_data_colision.txt", "w");

	for (unsigned int i = 0; i < robot_path.size(); i++)
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %lf %lf %lf\n", robot_path.at(i).x, robot_path.at(i).y, 1.0 * cos(robot_path.at(i).theta), 1.0 * sin(robot_path.at(i).theta), robot_path.at(i).theta, robot_path.at(i).phi, robot_path.at(i).time);
	for (unsigned int i = 0; i < collision_points.size(); i++)
		fprintf(gnuplot_data_lane, "%lf %lf\n", collision_points.at(i).x, collision_points.at(i).y);
//		fprintf(gnuplot_data_lane, "%lf %lf %lf %lf %lf %lf %lf\n", collision_points.at(i).x, collision_points.at(i).y, 1.0 * cos(collision_points.at(i).theta), 1.0 * sin(collision_points.at(i).theta), collision_points.at(i).theta, collision_points.at(i).phi, collision_points.at(i).time);

	fclose(gnuplot_data_file);
	fclose(gnuplot_data_lane);

//	fprintf(gnuplot_pipe, "unset arrow\nset arrow from %lf, %lf to %lf, %lf nohead\n",0, -60.0, 0, 60.0);

	fprintf(gnuplot_pipeMP, "plot "
			"'./gnuplot_data_path.txt' using 1:2 w lines title 'robot_path',"
			"'./gnuplot_data_colision.txt' using 1:2 title 'Collision'\n");

	fflush(gnuplot_pipeMP);
}


void
plot_state_goals(vector<carmen_ackerman_path_point_t> &pOTCP, vector<carmen_ackerman_path_point_t> &pLane,
		  vector<carmen_ackerman_path_point_t> &pSeed, vector<Pose> &pGoals)
{
//	plot data Table - Last TCP - Optmizer tcp - Lane
	//Plot Optmizer step tcp and lane?

	#define DELTA_T (1.0 / 40.0)

//	#define PAST_SIZE 300
	static bool first_time = true;
	static FILE *gnuplot_pipeMP;


	if (first_time)
	{
		first_time = false;

		gnuplot_pipeMP = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipeMP, "set xrange [0:70]\n");
		fprintf(gnuplot_pipeMP, "set yrange [-10:10]\n");
//		fprintf(gnuplot_pipe, "set y2range [-0.55:0.55]\n");
		fprintf(gnuplot_pipeMP, "set xlabel 'senconds'\n");
		fprintf(gnuplot_pipeMP, "set ylabel 'effort'\n");
//		fprintf(gnuplot_pipe, "set y2label 'phi (radians)'\n");
//		fprintf(gnuplot_pipe, "set ytics nomirror\n");
//		fprintf(gnuplot_pipe, "set y2tics\n");
		fprintf(gnuplot_pipeMP, "set tics out\n");
	}

	FILE *gnuplot_data_file = fopen("gnuplot_data.txt", "w");
	FILE *gnuplot_data_lane = fopen("gnuplot_data_lane.txt", "w");
	FILE *gnuplot_data_seed = fopen("gnuplot_data_seed.txt", "w");
	FILE *gnuplot_data_goals = fopen("gnuplot_data_goals.txt", "w");


	for (unsigned int i = 0; i < pOTCP.size(); i++)
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %lf %lf %lf\n", pOTCP.at(i).x, pOTCP.at(i).y, 1.0 * cos(pOTCP.at(i).theta), 1.0 * sin(pOTCP.at(i).theta), pOTCP.at(i).theta, pOTCP.at(i).phi, pOTCP.at(i).time);
	for (unsigned int i = 0; i < pLane.size(); i++)
		fprintf(gnuplot_data_lane, "%lf %lf %lf %lf %lf %lf %lf\n", pLane.at(i).x, pLane.at(i).y, 1.0 * cos(pLane.at(i).theta), 1.0 * sin(pLane.at(i).theta), pLane.at(i).theta, pLane.at(i).phi, pLane.at(i).time);
	for (unsigned int i = 0; i < pSeed.size(); i++)
		fprintf(gnuplot_data_seed, "%lf %lf\n", pSeed.at(i).x, pSeed.at(i).y);
	for (unsigned int i = 0; i < pGoals.size(); i++)
		fprintf(gnuplot_data_goals, "%lf %lf %lf %lf %lf\n", pGoals.at(i).x, pGoals.at(i).y, 1.0 * cos(pGoals.at(i).theta), 1.0 * sin(pGoals.at(i).theta), pGoals.at(i).theta);

	fclose(gnuplot_data_file);
	fclose(gnuplot_data_lane);
	fclose(gnuplot_data_seed);
	fclose(gnuplot_data_goals);

//	fprintf(gnuplot_pipe, "unset arrow\nset arrow from %lf, %lf to %lf, %lf nohead\n",0, -60.0, 0, 60.0);

	fprintf(gnuplot_pipeMP, "plot "
			"'./gnuplot_data.txt' using 1:2:3:4 w vec size 0.3, 10 filled title 'OTCP',"
			"'./gnuplot_data_lane.txt' using 1:2:3:4 w vec size 0.3, 10 filled title 'Lane',"
			"'./gnuplot_data_goals.txt' using 1:2:3:4 w vec size 0.3, 10 filled title 'Goals',"
			"'./gnuplot_data_seed.txt' using 1:2 with lines title 'Seed' axes x1y1\n");

	fflush(gnuplot_pipeMP);
}


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


void
compute_path_phis(vector<carmen_robot_and_trailers_path_point_t> *lane_in_local_pose)
{
	double L = GlobalState::robot_config.distance_between_front_and_rear_axles;
	for (unsigned int i = 0; i < lane_in_local_pose->size() - 1; i++)
	{
		double delta_theta = carmen_normalize_theta(lane_in_local_pose->at(i + 1).theta - lane_in_local_pose->at(i).theta);
		double l = DIST2D(lane_in_local_pose->at(i), lane_in_local_pose->at(i + 1));
		lane_in_local_pose->at(i).phi = atan(L * (delta_theta / l));
	}
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

//		compute_path_phis(lane_in_local_pose);
	}
	else
	{
		if (print_to_debug)
			printf(KGRN "+++++++++++++ ERRO MENSAGEM DA LANE POSES !!!!\n" RESET);
		return (false);
	}

	return (goal_in_lane);
}


void
filter_path_old(vector<carmen_ackerman_path_point_t> &path)
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
remove_some_poses_at_the_end_of_the_path(vector<carmen_ackerman_path_point_t> &path)
{
	double path_time = 0.0;

	while ((path.size() > 1) && (path_time < 1.0 / 10.0))
	{
		path_time += path[path.size() - 1].time;
		path.pop_back();
	}
}


void
filter_path(vector<carmen_robot_and_trailers_path_point_t> &path)
{
	if (path.size() < 1)
		return;

	unsigned int i, count;
	double path_time;

	path_time = 0.0;
	count = 0;
	for (i = 0; (i < path.size()) && (path_time < 0.6); i += 2)
	{
		if ((i + 1) < path.size())
		{
			path[i].time += path[i + 1].time;
			path_time += path[i].time;
			count++;
		}
	}
	for (i = 1; (i < path.size()) && (i < (count + 1)); i += 2)
		path.erase(path.begin() + i);

	path_time = 0.0;
	count = 0;
	for (i = 0; (i < path.size()) && (path_time < 0.6); i += 2)
	{
		if ((i + 1) < path.size())
		{
			path[i].time += path[i + 1].time;
			path_time += path[i].time;
			count++;
		}
	}
	for (i = 1; (i < path.size()) && (i < (count + 1)); i += 2)
		path.erase(path.begin() + i);

	if (GlobalState::ford_escape_online)
	{
		path_time = 0.0;
		count = 0;
		for (i = 0; (i < path.size()) && (path_time < 0.6); i += 2)
		{
			if ((i + 1) < path.size())
			{
				path[i].time += path[i + 1].time;
				path_time += path[i].time;
				count++;
			}
		}
		for (i = 1; (i < path.size()) && (i < (count + 1)); i += 2)
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
		for (double lat = 0.0; lat < 0.6; j++)
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


TrajectoryControlParameters
get_shorter_path(int &shorter_path, int num_paths, vector<vector<carmen_ackerman_path_point_t> > paths,
		const vector<TrajectoryControlParameters> otcps)
{
	double shorter_path_size = 1000.0;
	TrajectoryControlParameters best_otcp;

	for (int i = 0; i < num_paths; i++)
	{
		if ((paths[i].size() > 0) && (otcps[i].sf < shorter_path_size) && otcps[i].valid)
		{
			shorter_path_size = otcps[i].sf;
			shorter_path = i;
			best_otcp = otcps[i];
		}
	}
	return best_otcp;
}


double
get_intermediate_speed(double current_robot_pose_v, double v_goal, double dist_to_goal, double dist_to_curve)
{
	// https://www.wolframalpha.com/input/?i=solve+s%3Dg*(g-v)%2Fa%2B(v-g)*((g-v)%2F(2*a)))+for+a
	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*((g-v)%2Fa)%2B0.5*a*((g-v)%2Fa)%5E2+for+g

	if (dist_to_goal > dist_to_curve)
		return v_goal;

	double v0 = current_robot_pose_v;
	double a = (v_goal * v_goal - v0 * v0) / (2.0 * dist_to_curve);
	double sqrt_val = 2.0 * a * dist_to_goal + v0 * v0;
	double vg = v_goal;
	if (sqrt_val > 0.0)
		vg = sqrt(sqrt_val);
	if (vg < v_goal)
		vg = v_goal;

//	static double first_time = 0.0;
//	double t = carmen_get_time();
//	if (first_time == 0.0)
//		first_time = t;
//	printf("t %.3lf, v0 %.1lf, va %.1lf, a %.3lf, vg %.2lf, dg %.1lf, da %.1lf\n", t - first_time, v0, va, a, vg, dg, da);
//	printf("t %.3lf, v0 %.1lf, a %.3lf, vg %.2lf, dg %.1lf, tt %.3lf\n", t - first_time, v0, a, vg, dg, (vg - v0) / a);

	return (vg);
}


bool
get_path_from_optimized_tcp(vector<carmen_robot_and_trailers_path_point_t> &path,
		vector<carmen_robot_and_trailers_path_point_t> &path_local,
		TrajectoryControlParameters otcp,
		TrajectoryDimensions td,
		carmen_robot_and_trailers_pose_t *localizer_pose)
{
	if (GlobalState::use_mpc)
		path = simulate_car_from_parameters(td, otcp, td.v_i, td.trailer_theta_i, 0.025);
	else if (use_unity_simulator)
		path = simulate_car_from_parameters(td, otcp, td.v_i, td.trailer_theta_i, 0.02);
	else if (GlobalState::eliminate_path_follower)
		path = simulate_car_from_parameters(td, otcp, td.v_i, td.trailer_theta_i, 0.02);
	else
		path = simulate_car_from_parameters(td, otcp, td.v_i, td.trailer_theta_i);

	path_local = path;
	if (path_has_loop(td.dist, otcp.sf))
	{
		printf(KRED "+++++++++++++ Path has loop...\n" RESET);
		return (false);
	}

//	if (path_has_collision_or_phi_exceeded(path))
//		return (false);
	if (path_has_collision_or_phi_exceeded(path))
		GlobalState::path_has_collision_or_phi_exceeded = true;
	else
		GlobalState::path_has_collision_or_phi_exceeded = false;

	move_path_to_current_robot_pose(path, localizer_pose);

//	printf("\n* MPP path\n");
//	for (unsigned int i = 0; (i < path.size()) && (i < 15); i++)
//	{
//		printf("v %2.2lf, phi %5.3lf, t %5.3lf, x %5.3lf, y %5.3lf, theta %5.3lf\n",
//				path[i].v, path[i].phi, path[i].time,
//				path[i].x - localizer_pose->x, path[i].y - localizer_pose->y, path[i].theta);
//	}
//	fflush(stdout);

	// Para evitar que o fim do path bata em obstáculos devido a atrazo na propagacao da posicao atual deles
//	remove_some_poses_at_the_end_of_the_path(path);

//	if (GlobalState::use_mpc)
//		apply_system_latencies(path);
//	else
//		filter_path(path);
	if (!GlobalState::use_mpc && !use_unity_simulator && !GlobalState::eliminate_path_follower)
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
//	if (too_different)
//		printf("TOO DIFFERENT!!!\n");

	return (too_different);
}

bool
goal_is_behind_car(Pose *localizer_pose, Pose *goal_pose)
{//funcao tem que ser melhorada. Usar coordenadas polares pode ser melhor.
	SE2 robot_pose(localizer_pose->x, localizer_pose->y, localizer_pose->theta);
	SE2 goal_in_world_reference(goal_pose->x, goal_pose->y, goal_pose->theta);
	SE2 goal_in_car_reference = robot_pose.inverse() * goal_in_world_reference;
	double goal_x = goal_in_car_reference[0];
	double goal_theta = goal_in_car_reference[2];

	if(goal_x <= 0.0 && fabs(goal_theta) < M_PI_2)
		return true;

	return false;
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
			td.trailer_theta_i[z] = carmen_normalize_theta(-convert_theta1_to_beta(localizer_pose->theta, localizer_pose->trailer_theta[z])); // Só funcionou dessa forma, utilizando o beta nessa linha com o -
	}


	SE2 robot_pose(localizer_pose->x, localizer_pose->y,localizer_pose->theta);
	SE2 goal_in_world_reference(goal_pose->x, goal_pose->y, goal_pose->theta);
	SE2 goal_in_car_reference = robot_pose.inverse() * goal_in_world_reference;
	td.goal_pose.x = goal_in_car_reference[0];
	td.goal_pose.y = goal_in_car_reference[1];
	td.goal_pose.theta = goal_in_car_reference[2];

	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
	{
		double goal_beta = convert_theta1_to_beta(goal_pose->theta, goal_pose->trailer_theta[z]); // Esse beta do goal_pose é trailer_theta. Aqui é feito a conversão para o beta para conseguir trocar o trailer_theta de referência
		td.goal_pose.trailer_theta[z] = convert_beta_to_theta1(goal_in_car_reference[2], goal_beta);
	}

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
	static double last_timestamp = 0.0;
	bool goal_in_lane = false;

	if (first_time || !GlobalState::following_path)
	{
		previous_good_tcp.valid = false;
		first_time = false;
		last_timestamp = path_goals_and_annotations_message->timestamp;
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

#ifdef EXPERIMENT_DATA
		save_experiment_data(path_goals_and_annotations_message, localizer_pose,
						 detailed_lane, last_odometry, target_v);
#endif

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
//		printf("%lf   %lf\n", path[path.size() - 1].beta, goal_pose->beta);
		fflush(stdout);

		previous_good_tcp = otcp;
		last_timestamp = path_goals_and_annotations_message->timestamp;
	}
	else
	{
		if ((path_goals_and_annotations_message->timestamp - last_timestamp) > 0.5)
			previous_good_tcp.valid = false;

		paths.clear();
	}
	//SALVAR TAMBEM OS PATHS PARA VER PRA CADA POSE E GOAL QUAL FOI A SAIDA

	return (paths);
}

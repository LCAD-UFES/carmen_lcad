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


using namespace g2o;

int print_and_plot = 0;

//-----------Funcoes para extrair dados do Experimento------------------------
double
dist(carmen_ackerman_path_point_t v, carmen_ackerman_path_point_t w)
{
    return sqrt((carmen_square(v.x - w.x) + carmen_square(v.y - w.y)));
}


double
get_distance_between_point_to_line(carmen_ackerman_path_point_t p1,
        carmen_ackerman_path_point_t p2,
        carmen_ackerman_path_point_t robot)
{
    //https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    double delta_x = p2.x - p1.x;
    double delta_y = p2.y - p1.y;
    double d = sqrt(delta_x * delta_x + delta_y * delta_y);
    double x2y1 =  p2.x * p1.y;
    double y2x1 =  p2.y * p1.x;

    if (d < 0.0000001)
        return dist(p2, robot);

    return abs((delta_y * 0.0) - (delta_x * 0.0) + x2y1 - y2x1) / d;

}


void
get_points2(vector<carmen_ackerman_path_point_t> &detailed_goal_list, int &index_p1, int &index_p2, int &mais_proxima)
{

    double d = sqrt(pow(detailed_goal_list.at(0).x, 2) + pow(detailed_goal_list.at(0).y, 2));
    double d2 = sqrt(pow(detailed_goal_list.at(2).x, 2) + pow(detailed_goal_list.at(2).y, 2));
    double centro = sqrt(pow(detailed_goal_list.at(1).x, 2) + pow(detailed_goal_list.at(1).y, 2));
    if (d < d2)
    {
        index_p1 = 0;
        index_p2 = 1;
        mais_proxima = d;
        if(centro < d)
            mais_proxima = centro;
    }
    else
    {
        index_p1 = 1;
        index_p2 = 2;
        mais_proxima = d2;
        if(centro < d2)
            mais_proxima = centro;
    }
}


void
save_experiment_data(carmen_behavior_selector_road_profile_message *goal_list_message,
					Pose *localizer_pose, vector<carmen_ackerman_path_point_t> &detailed_lane,
					const vector<Command> &lastOdometryVector)
{
	if (detailed_lane.size() > 0)
	{
		carmen_ackerman_path_point_t localize;
		localize.x = 0.0;
		localize.y = 0.0;
		//Metric evaluation
		int index1;
		int index2;
		int mais_proxima;
		get_points2(detailed_lane,index1, index2,mais_proxima);
		//      printf("%lf %lf \n", detailed_goal_list.at(index1).x, detailed_goal_list.at(index2).x);
		double distance_metric = get_distance_between_point_to_line(detailed_lane.at(index1), detailed_lane.at(index2), localize);
		//      printf("%lf \n", distance_metric);
		double x_rddf = localizer_pose->x + detailed_lane.at(mais_proxima).x * cos(localizer_pose->theta) - detailed_lane.at(mais_proxima).y * sin(localizer_pose->theta);
		double y_rddf = localizer_pose->y + detailed_lane.at(mais_proxima).x * sin(localizer_pose->theta) + detailed_lane.at(mais_proxima).y * cos(localizer_pose->theta);
		double theta_rddf = detailed_lane.at(mais_proxima).theta + localizer_pose->theta;
		double volante_rddf_theta = atan2(detailed_lane.at(index1).y - detailed_lane.at(index2).y , detailed_lane.at(index1).x - detailed_lane.at(index2).x);
		double erro_theta = abs(volante_rddf_theta - localizer_pose->theta);
		//          1-Localise_x 2-Localise_y 3-Localise_theta 4-velocity 5-phi 6-rddf_x 7-rddf_y 8-rddf_theta 9-rddf_velocity 10-rddf_phi 11-lateralDist 12-volante 13-erro_theta 14-Timestamp
		fprintf(stderr, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \n", localizer_pose->x, localizer_pose->y, localizer_pose->theta,
				lastOdometryVector[0].v, lastOdometryVector[0].phi, x_rddf, y_rddf, theta_rddf, detailed_lane.at(mais_proxima).v,
				detailed_lane.at(mais_proxima).phi, distance_metric, volante_rddf_theta, erro_theta, goal_list_message->timestamp);

	}
}

//------------------------------------------------------------


void
plot_state(vector<carmen_ackerman_path_point_t> &pOTCP, vector<carmen_ackerman_path_point_t> &pLane,
		  vector<carmen_ackerman_path_point_t> &pSeed)
{
//	plot data Table - Last TCP - Optmizer tcp - Lane
	//Plot Optmizer step tcp and lane?

	#define DELTA_T (1.0 / 40.0)

//	#define PAST_SIZE 300
//	static vector<carmen_ackerman_path_point_t> pOTCP_vector;
//	static vector<carmen_ackerman_path_point_t> pLane_vector;
//	static vector<carmen_ackerman_path_point_t> pSeed_vector;
//	static vector<double> timestamp_vector;
	static bool first_time = true;
//	static double first_timestamp;
	static FILE *gnuplot_pipeMP;

//	double t = carmen_get_time();

	if (first_time)
	{
//		first_timestamp = t;
		first_time = false;

		gnuplot_pipeMP = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipeMP, "set xrange [0:40]\n");
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

	for (unsigned int i = 0; i < pOTCP.size(); i++)
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %lf %lf %lf\n", pOTCP.at(i).x, pOTCP.at(i).y, 1.0 * cos(pOTCP.at(i).theta), 1.0 * sin(pOTCP.at(i).theta), pOTCP.at(i).theta, pOTCP.at(i).phi, pOTCP.at(i).time);
	for (unsigned int i = 0; i < pLane.size(); i++)
		fprintf(gnuplot_data_lane, "%lf %lf %lf %lf %lf %lf %lf\n", pLane.at(i).x, pLane.at(i).y, 1.0 * cos(pLane.at(i).theta), 1.0 * sin(pLane.at(i).theta), pLane.at(i).theta, pLane.at(i).phi, pLane.at(i).time);

	for (unsigned int i = 0; i < pSeed.size(); i++)
		fprintf(gnuplot_data_seed, "%lf %lf\n", pSeed.at(i).x, pSeed.at(i).y);

	fclose(gnuplot_data_file);
	fclose(gnuplot_data_lane);
	fclose(gnuplot_data_seed);

//	fprintf(gnuplot_pipe, "unset arrow\nset arrow from %lf, %lf to %lf, %lf nohead\n",0, -60.0, 0, 60.0);

	fprintf(gnuplot_pipeMP, "plot "
			"'./gnuplot_data.txt' using 1:2:3:4 w vec size  0.3, 10 filled title 'OTCP',"
			"'./gnuplot_data_lane.txt' using 1:2:3:4 w vec size  0.3, 10 filled title 'Lane',"
			"'./gnuplot_data_seed.txt' using 1:2 with lines title 'Seed' axes x1y1\n");

	fflush(gnuplot_pipeMP);
}

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
move_lane_to_robot_reference_system(Pose *localizer_pose, carmen_behavior_selector_road_profile_message *goal_list_message,
		Pose *goal_pose, vector<carmen_ackerman_path_point_t> *lane_in_local_pose)
{
	double last_dist = DBL_MAX;
	double dist = 0.0;
	double close_to_goal_x;
	double close_to_goal_y;
	SE2 robot_pose(localizer_pose->x, localizer_pose->y, localizer_pose->theta);
	SE2 goal_in_world_reference(goal_pose->x, goal_pose->y, goal_pose->theta);
	SE2 goal_in_car_reference = robot_pose.inverse() * goal_in_world_reference;
	double goal_x = goal_in_car_reference[0];
	double goal_y = goal_in_car_reference[1];

	if ((goal_list_message->number_of_poses < 2 || goal_list_message->number_of_poses > 250))
		return false;

	vector<carmen_ackerman_path_point_t> poses_back;
	carmen_ackerman_path_point_t local_reference_lane_point;

	if ((goal_list_message->number_of_poses_back > 2))
	{
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
		close_to_goal_x = fabs(goal_x - local_reference_lane_point.x);
		close_to_goal_y = fabs(goal_y - local_reference_lane_point.y);

		if ((close_to_goal_x < 0.6) && (close_to_goal_y < 0.6))
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
	double delta_x, delta_y, delta_theta, distance;
	int i;
	static bool first = true;

	distance = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));

	double distance_between_goals = 0.1;
	int num_points = distance / distance_between_goals;

	delta_x = (p2.x - p1.x) / num_points;
	delta_y = (p2.y - p1.y) / num_points;
	delta_theta = carmen_normalize_theta((p2.theta - p1.theta)) / num_points;

	carmen_ackerman_path_point_t new_point = {p1.x, p1.y, p1.theta, p1.v, p1.phi, 0.0};


	for (i = 0; i < num_points; i++)
	{
		new_point.x = p1.x + i * delta_x;
		new_point.y = p1.y + i * delta_y;
		new_point.theta = carmen_normalize_theta(p1.theta + i * delta_theta);

		if (new_point.x == 0.0 && first)
			first = false;
		if (new_point.x > 0.0)
		{
			if (first)
			{
				// Insert the first pose (car pose) to path_planner lane
				carmen_ackerman_path_point_t first_point = {0.0, 0.0, 0.0, p1.v, p1.phi, 0.0};
				detailed_lane.push_back(first_point);
				first = false;
			}

			detailed_lane.push_back(new_point);
		}

	}
}


void
copy_starting_nearest_point_of_zero(vector<carmen_ackerman_path_point_t> &detailed_lane, vector<carmen_ackerman_path_point_t> &temp_detail)
{
	detailed_lane.clear();

	for (unsigned int i = 1; i < temp_detail.size(); i++)
	{
		if (temp_detail.at(i).x > 0.0)
		{
			double distance1 = sqrt((carmen_square(temp_detail.at(i-1).x - 0.0) + carmen_square(temp_detail.at(i-1).y - 0.0)));
			double distance2 = sqrt((carmen_square(temp_detail.at(i).x - 0.0) + carmen_square(temp_detail.at(i).y - 0.0)));
			if ((distance1 < distance2))
				i--;
			int k = 0;
			for (unsigned int j = i; j < temp_detail.size(); j++ , k++)
				detailed_lane.push_back(temp_detail.at(j));
			break;
		}
	}
}


bool
build_detailed_path_lane(vector<carmen_ackerman_path_point_t> *lane_in_local_pose, vector<carmen_ackerman_path_point_t> &detailed_lane)
{
	if (lane_in_local_pose->size() > 2 && lane_in_local_pose->size() < 200)
	{
		for (unsigned int i = 0; i < (lane_in_local_pose->size() - 1); i++)
		{
			add_points_to_goal_list_interval(lane_in_local_pose->at(i), lane_in_local_pose->at(i+1), detailed_lane);
			if (detailed_lane.size() > 250)
			{
				detailed_lane.clear();
				return false;
			}
		}
		detailed_lane.push_back(lane_in_local_pose->back());
	}
	else
	{
		if(print_and_plot)
			printf(KGRN "+++++++++++++ ERRO MENSAGEM DA LANE POSES !!!!\n" RESET);
		detailed_lane.clear();
		return (false);
	}
	return (true);
}


bool
build_detailed_rddf_lane(vector<carmen_ackerman_path_point_t> *lane_in_local_pose, vector<carmen_ackerman_path_point_t> &detailed_lane)
{
	if (lane_in_local_pose->size() > 2)
	{
		vector<carmen_ackerman_path_point_t> temp_detail;
		for (unsigned int i = 0; i < (lane_in_local_pose->size() - 1); i++)
			add_points_to_goal_list_interval(lane_in_local_pose->at(i), lane_in_local_pose->at(i+1), temp_detail);

		temp_detail.push_back(lane_in_local_pose->back());
		copy_starting_nearest_point_of_zero(detailed_lane, temp_detail);
	}
	else
	{
		if(print_and_plot)
			printf(KGRN "+++++++++++++ ERRO MENSAGEM DA LANE POSES !!!!\n" RESET);
		detailed_lane.clear();
		return (false);
	}
	return (true);
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
	carmen_point_t localizer = {GlobalState::localizer_pose->x, GlobalState::localizer_pose->y, GlobalState::localizer_pose->theta};

	for (unsigned int i = 0; i < path.size(); i += 1)
	{
		if ((path[i].phi > GlobalState::robot_config.max_phi) ||
				(path[i].phi < -GlobalState::robot_config.max_phi))
			printf("---------- PHI EXCEEDED THE MAX_PHI!!!!\n");

		carmen_point_t point_to_check = {path[i].x, path[i].y, path[i].theta};
		if (GlobalState::distance_map != NULL)
			proximity_to_obstacles_for_path += carmen_obstacle_avoider_compute_car_distance_to_closest_obstacles(&localizer,
					point_to_check, GlobalState::robot_config, GlobalState::distance_map, circle_radius);
		else
			return (true);
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


TrajectoryLookupTable::TrajectoryControlParameters
get_shorter_path(int &shorter_path, int num_paths, vector<vector<carmen_ackerman_path_point_t> > paths,
		const vector<TrajectoryLookupTable::TrajectoryControlParameters> otcps)
{
	double shorter_path_size = 1000.0;
	TrajectoryLookupTable::TrajectoryControlParameters best_otcp;

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


void
limit_maximum_centripetal_acceleration_old(vector<carmen_ackerman_path_point_t> &path)
{

	double max_centripetal_acceleration = 0.0;

	for (unsigned int i = 0; i < path.size(); i += 1)
	{
		if (fabs(path[i].phi) > 0.001)
		{
			double radius_of_curvature = GlobalState::robot_config.distance_between_front_and_rear_axles / fabs(tan(path[i].phi));
			double centripetal_acceleration = (path[i].v * path[i].v) / radius_of_curvature;
			if (centripetal_acceleration > max_centripetal_acceleration)
				max_centripetal_acceleration = centripetal_acceleration;
		}
	}

//	printf("max_c_a = %lf\n", max_centripetal_acceleration);

	if (max_centripetal_acceleration > GlobalState::robot_max_centripetal_acceleration)
	{
		double reduction_factor = GlobalState::robot_max_centripetal_acceleration / max_centripetal_acceleration;

		for (unsigned int i = 0; i < path.size(); i += 1)
		{
			path[i].v *= reduction_factor;
			path[i].time *= 1.0 / reduction_factor;
		}
	}
}


void
limit_maximum_centripetal_acceleration(double &target_v, vector<carmen_ackerman_path_point_t> &path)
{
	double max_centripetal_acceleration = 0.0;
	static double reduction_factor = 0.0;

	for (unsigned int i = 0; (i < path.size() - 1) && (path.size() != 0); i += 1)
	{
		double L = GlobalState::robot_config.distance_between_front_and_rear_axles;
		double delta_theta = path[i + 1].theta - path[i].theta;
		double l = dist(path[i], path[i + 1]);
		path[i].phi = L * atan(delta_theta / l);

		if (fabs(path[i].phi) > 0.001)
		{
			double radius_of_curvature = L / fabs(tan(path[i].phi));
			double centripetal_acceleration = (path[i].v * path[i].v) / radius_of_curvature;
			if (centripetal_acceleration > max_centripetal_acceleration)
				max_centripetal_acceleration = centripetal_acceleration;
		}
	}

	if (max_centripetal_acceleration > GlobalState::robot_max_centripetal_acceleration)
	{
		reduction_factor += 0.2 * ((GlobalState::robot_max_centripetal_acceleration / max_centripetal_acceleration) - reduction_factor);
		target_v *= reduction_factor;
		// printf("redution %lf, target_v %lf\n", reduction_factor, target_v);
	}
}


bool
get_path_from_optimized_tcp(vector<carmen_ackerman_path_point_t> &path,
		vector<carmen_ackerman_path_point_t> &path_local,
		TrajectoryLookupTable::TrajectoryControlParameters otcp,
		TrajectoryLookupTable::TrajectoryDimensions td,
		Pose *localizer_pose)
{
	if (GlobalState::use_mpc)
		path = simulate_car_from_parameters(td, otcp, td.v_i, td.phi_i, false, 0.025);
	else
		path = simulate_car_from_parameters(td, otcp, td.v_i, td.phi_i, false);
	path_local = path;
	if (path_has_loop(td.dist, otcp.sf))
	{
		printf(KRED "+++++++++++++ Path has loop...\n" RESET);
		return (false);
	}

	if (path_has_collision_or_phi_exceeded(path))
		return (false);

	move_path_to_current_robot_pose(path, localizer_pose);

//	limit_maximum_centripetal_acceleration_old(path);

//	if (GlobalState::use_mpc)
//		apply_system_latencies(path);
//	else
//		filter_path(path);
	if (!GlobalState::use_mpc)
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
goal_is_behide_car(Pose *localizer_pose, Pose *goal_pose)
{//funcao tem que ser melhorada. Usar coordenadas polares pode ser melhor.
	SE2 robot_pose(localizer_pose->x, localizer_pose->y, localizer_pose->theta);
	SE2 goal_in_world_reference(goal_pose->x, goal_pose->y, goal_pose->theta);
	SE2 goal_in_car_reference = robot_pose.inverse() * goal_in_world_reference;
	double goal_x = goal_in_car_reference[0];

	if(goal_x <= 0.0)
		return true;

	return false;
}


void
compute_paths(const vector<Command> &lastOdometryVector, vector<Pose> &goalPoseVector, double target_v,
		Pose *localizer_pose, vector<vector<carmen_ackerman_path_point_t> > &paths,
		carmen_behavior_selector_road_profile_message *goal_list_message)
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

//	if(goal_is_behide_car(localizer_pose, &goalPoseVector[0]))
//	{
//		printf("goal is behide the car\n");
//		return;
//	}

	bool goal_in_lane = false;
	goal_in_lane = move_lane_to_robot_reference_system(localizer_pose, goal_list_message, &goalPoseVector[0], &lane_in_local_pose);

	if (!goal_in_lane)
		lane_in_local_pose.clear();

	if (GlobalState::use_path_planner || GlobalState::use_tracker_goal_and_lane)
		build_detailed_path_lane(&lane_in_local_pose, detailed_lane);
	else
		build_detailed_rddf_lane(&lane_in_local_pose, detailed_lane);

	// Aberto: @@@ Esta funcao escreve no phi de detailed_lane
	limit_maximum_centripetal_acceleration(target_v, detailed_lane);

/***************************************
 * Funcao para extrair dados para artigo
 *	save_experiment_data(goal_list_message, localizer_pose,
						 detailed_lane, lastOdometryVector);
 *	return;
******************************************/

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
				if (false)//goal_pose_vector_too_different(goalPoseVector.at(0), *localizer_pose))
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
				vector<carmen_ackerman_path_point_t> path_local;

				//TODO Descomentar para usar o plot!
				vector<carmen_ackerman_path_point_t> pathSeed;
				if (print_and_plot)
				{
					pathSeed = simulate_car_from_parameters(td, tcp, lastOdometryVector[0].v, lastOdometryVector[0].phi, false, 0.025);
				}

				if (!get_path_from_optimized_tcp(path, path_local, otcp, td, localizer_pose))
					continue;

				//TODO Gnuplot
				if (print_and_plot)
					plot_state(path_local,detailed_lane,pathSeed);

				paths[j + i * lastOdometryVector.size()] = path;
				otcps[j + i * lastOdometryVector.size()] = otcp;

				has_valid_path = true;
				break;
			}
			else
			{
				otcps[j + i * lastOdometryVector.size()] = otcp;
				// printf(KYEL "+++++++++++++ Could NOT optimize !!!!\n" RESET);
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

}


vector<vector<carmen_ackerman_path_point_t> >
compute_path_to_goal(Pose *localizer_pose, Pose *goal_pose, Command last_odometry,
		double target_v, carmen_behavior_selector_road_profile_message *goal_list_message)
{
	vector<Command> lastOdometryVector;
	vector<Pose> goalPoseVector;
	vector<vector<carmen_ackerman_path_point_t>> paths;

//	double i_time = carmen_get_time();

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
		Pose newPose = *goal_pose;
		newPose.x += 0.3 * (double) magicSignals[i] * cos(carmen_normalize_theta((goal_pose->theta) - carmen_degrees_to_radians(90.0)));
		newPose.y += 0.3 * (double) magicSignals[i] * sin(carmen_normalize_theta((goal_pose->theta) - carmen_degrees_to_radians(90.0)));
		goalPoseVector.push_back(newPose);
	}

	paths.resize(lastOdometryVector.size() * goalPoseVector.size());

	compute_paths(lastOdometryVector, goalPoseVector, target_v, localizer_pose, paths, goal_list_message);

//	printf("%ld plano(s), tp = %lf\n", paths.size(), carmen_get_time() - i_time);
//	fflush(stdout);

	return (paths);
}

/*
 * model_predictive_planner_optimizer.h
 *
 *  Created on: Jun 23, 2016
 *      Author: lcad
 */

#ifndef MODEL_PREDICTIVE_PLANNER_OPTIMIZER_H_
#define MODEL_PREDICTIVE_PLANNER_OPTIMIZER_H_

#include "trajectory_lookup_table.h"
#include <vector>
using namespace std;

struct ObjectiveFunctionParams
{
	double target_v, suitable_acceleration, suitable_tt;
	double distance_by_index;
	double theta_by_index;
	double d_yaw_by_index;
	TrajectoryLookupTable::TrajectoryControlParameters *tcp_seed;
	TrajectoryLookupTable::TrajectoryDimensions *target_td;
	vector<carmen_ackerman_path_point_t> detailed_lane;
	vector<unsigned int> path_point_nearest_to_lane;
	unsigned int path_size;
	bool use_lane;
	int optimize_time;
	double plan_cost;
};

#define OPTIMIZE_ACCELERATION 	0
#define OPTIMIZE_TIME 			1
#define OPTIMIZE_DISTANCE 		2

TrajectoryLookupTable::TrajectoryControlParameters get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDiscreteDimensions &tdd, double target_v, vector<carmen_ackerman_path_point_t> optimized_path);

TrajectoryLookupTable::TrajectoryControlParameters get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd, double target_v);

TrajectoryLookupTable::TrajectoryControlParameters get_complete_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, vector<carmen_ackerman_path_point_t> detailed_lane,
		bool use_lane, bool has_previous_good_tcp);

#endif /* MODEL_PREDICTIVE_PLANNER_OPTIMIZER_H_ */

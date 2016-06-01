/*
 * trajectory_lookup_table.cpp
 *
 *  Created on: Jun 10, 2015
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
#include "trajectory_lookup_table.h"

#include "g2o/types/slam2d/se2.h"

using namespace g2o;

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

//TODO
//#define DEBUG_LANE

struct ObjectiveFunctionParams
{
	double target_v, suitable_acceleration;
	double distance_by_index;
	double theta_by_index;
	double d_yaw_by_index;
	TrajectoryLookupTable::TrajectoryControlParameters *tcp_seed;
	TrajectoryLookupTable::TrajectoryDimensions *target_td;
	vector<carmen_ackerman_path_point_t> detailed_goal_list;
	vector<unsigned int> nearest_path_point;
	double lane_sf;
	unsigned int path_size;
};

#define SYSTEM_DELAY 0.7

TrajectoryLookupTable::TrajectoryControlParameters trajectory_lookup_table[N_DIST][N_THETA][N_D_YAW][N_I_PHI][N_I_V];
TrajectoryLookupTable::CarLatencyBuffer g_car_latency_buffer;

double g_last_lane_timestamp = 0.0;
vector<double> step_sf;
vector<double> lane_step_sf;

bool use_lane = true;
bool use_obstacles = true;


TrajectoryLookupTable::TrajectoryLookupTable(int update_lookup_table)
{
	if (!load_trajectory_lookup_table())
		build_trajectory_lookup_table();

	if (update_lookup_table)
		update_lookup_table_entries();

	evaluate_trajectory_lookup_table();
	std::cout << "Finished loading/creating the trajectory lookup table!\n";
}


bool
has_valid_discretization(TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd)
{
	if ((tdd.dist < N_DIST) &&
		(tdd.theta < N_THETA) &&
		(tdd.d_yaw < N_D_YAW) &&
		(tdd.phi_i < N_I_PHI) &&
		(tdd.v_i < N_I_V))
		return (true);
	else
		return (false);
}


TrajectoryLookupTable::TrajectoryControlParameters
search_lookup_table(TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd)
{
    // TODO: pegar a media de todas as leituras ponderada pela distancia para o td.
    // Tem que passar o td ao inves do tdd.
	TrajectoryLookupTable::TrajectoryControlParameters tcp;
	tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
	if (tcp.valid)
		return (tcp);

	tdd.dist += 1;
	if (has_valid_discretization(tdd))
	{
		tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
		if (tcp.valid)
			return (tcp);
	}
	tdd.dist -= 2;
	if (has_valid_discretization(tdd))
	{
		tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
		if (tcp.valid)
			return (tcp);
	}
	tdd.dist += 1;

	tdd.theta += 1;
	if (has_valid_discretization(tdd))
	{
		tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
		if (tcp.valid)
			return (tcp);
	}
	tdd.theta -= 2;
	if (has_valid_discretization(tdd))
	{
		tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
		if (tcp.valid)
			return (tcp);
	}
	tdd.theta += 1;

	tdd.d_yaw += 1;
	if (has_valid_discretization(tdd))
	{
		tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
		if (tcp.valid)
			return (tcp);
	}
	tdd.d_yaw -= 2;
	if (has_valid_discretization(tdd))
	{
		tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
		if (tcp.valid)
			return (tcp);
	}
	tdd.d_yaw += 1;

	tdd.phi_i += 1;
	if (has_valid_discretization(tdd))
	{
		tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
		if (tcp.valid)
			return (tcp);
	}
	tdd.phi_i -= 2;
	if (has_valid_discretization(tdd))
	{
		tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
		if (tcp.valid)
			return (tcp);
	}
	tdd.phi_i += 1;

	tdd.v_i += 1;
	if (has_valid_discretization(tdd))
	{
		tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
		if (tcp.valid)
			return (tcp);
	}
	tdd.v_i -= 2;
	if (has_valid_discretization(tdd))
	{
		tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
		if (tcp.valid)
			return (tcp);
	}
	tdd.v_i += 1;

	tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
	return (tcp);
}



TrajectoryLookupTable::TrajectoryControlParameters
search_lookup_table_loop(TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd)
{
	// @@@ Alberto: Esta função é muito lenta... Voltei para a search_lookup_table().
    // TODO: pegar a media de todas as leituras ponderada pela distancia para o td.
    // Tem que passar o td ao inves do tdd.
	TrajectoryLookupTable::TrajectoryControlParameters tcp;
	tcp.valid = false;
	vector<int> signalSum = {0 ,-1, 2, -2, 4};

	for (unsigned int i = 0; i < signalSum.size(); i++)
		for (unsigned int j = 0; j < signalSum.size(); j++)
			for (unsigned int k = 0; k < signalSum.size(); k++)
				for (unsigned int l = 0; l < signalSum.size(); l++)
					for (unsigned int m = 0; m < signalSum.size(); m++)
					{
						tdd.dist += signalSum[i];
						tdd.theta += signalSum[j];
						tdd.d_yaw += signalSum[k];
						tdd.phi_i += signalSum[l];
						tdd.v_i += signalSum[m];

						if (has_valid_discretization(tdd))
						{
							tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
							if (tcp.valid)
								return (tcp);
						}
					}
	return (tcp);
}



void
TrajectoryLookupTable::evaluate_trajectory_lookup_table()
{
	double parameter_count[6][100];
	double parameter_valid[6][100];
	double total_count = 0.0 ;
	double total_valid = 0.0;

	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 100; j++)
		{
			parameter_count[i][j] = 0.0;
			parameter_valid[i][j] = 0.0;
		}

	for (int i = 0; i < N_DIST; i++)
		for (int j = 0; j < N_THETA; j++)
			for (int k = 0; k < N_D_YAW; k++)
				for (int l = 0; l < N_I_PHI; l++)
					for (int m = 0; m < N_I_V; m++)
					{
						parameter_count[0][i] += 1.0;
						parameter_count[1][j] += 1.0;
						parameter_count[2][k] += 1.0;
						parameter_count[3][l] += 1.0;
						parameter_count[4][m] += 1.0;
						total_count += 1.0;
						if (trajectory_lookup_table[i][j][k][l][m].valid)
						{
							parameter_valid[0][i] += 1.0;
							parameter_valid[1][j] += 1.0;
							parameter_valid[2][k] += 1.0;
							parameter_valid[3][l] += 1.0;
							parameter_valid[4][m] += 1.0;
							total_valid += 1.0;
						}
						else
						{
							TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd;
							tdd.d_yaw = k;
							tdd.dist = i;
							tdd.phi_i = l;
							tdd.theta = j;
							tdd.v_i = m;
							TrajectoryLookupTable::TrajectoryControlParameters tcp = search_lookup_table(tdd);
							if (tcp.valid)
							{
								parameter_valid[0][i] += 1.0;
								parameter_valid[1][j] += 1.0;
								parameter_valid[2][k] += 1.0;
								parameter_valid[3][l] += 1.0;
								parameter_valid[4][m] += 1.0;
								total_valid += 1.0;
							}
						}
					}

	for (int i = 0; i < N_DIST; i++)
		printf("N_DIST %02d = %3.3lf%%\n", i, 100.0 * (parameter_valid[0][i] / parameter_count[0][i]));
	printf("\n");
	for (int j = 0; j < N_THETA; j++)
		printf("N_THETA %02d = %3.3lf%%\n", j, 100.0 * (parameter_valid[1][j] / parameter_count[1][j]));
	printf("\n");
	for (int k = 0; k < N_D_YAW; k++)
		printf("N_D_YAW %02d = %3.3lf%%\n", k, 100.0 * (parameter_valid[2][k] / parameter_count[2][k]));
	printf("\n");
	for (int l = 0; l < N_I_PHI; l++)
		printf("N_I_PHI %02d = %3.3lf%%\n", l, 100.0 * (parameter_valid[3][l] / parameter_count[3][l]));
	printf("\n");
	for (int m = 0; m < N_I_V; m++)
		printf("N_I_V %02d = %3.3lf%%\n", m, 100.0 * (parameter_valid[4][m] / parameter_count[4][m]));
	printf("\n");

	printf("total = %3.3lf%%\n", 100.0 * (total_valid / total_count));
}


void
init_trajectory_lookup_table()
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp;

	tcp.valid = false;
	tcp.velocity_profile = CONSTANT_PROFILE;
	tcp.v0 = 0.0;
	tcp.vt = 0.0;
	tcp.vf = 0.0;
	tcp.a0 = 0.0;
	tcp.af = 0.0;
	tcp.t0 = 0.0;
	tcp.tt = 0.0;
	tcp.tf = 0.0;
	tcp.k1 = 0.0;
	tcp.k2 = 0.0;
	tcp.k3 = 0.0;
	tcp.has_k3 = false;
	tcp.sf = 0.0;

	for (int i = 0; i < N_DIST; i++)
		for (int j = 0; j < N_THETA; j++)
			for (int k = 0; k < N_D_YAW; k++)
				for (int l = 0; l < N_I_PHI; l++)
					for (int m = 0; m < N_I_V; m++)
						trajectory_lookup_table[i][j][k][l][m] = tcp;
}


void
save_trajectory_lookup_table()
{
	FILE *tlt_f;

	tlt_f = fopen("trajectory_lookup_table.bin", "w");

	for (int i = 0; i < N_DIST; i++)
		for (int j = 0; j < N_THETA; j++)
			for (int k = 0; k < N_D_YAW; k++)
				for (int l = 0; l < N_I_PHI; l++)
					for (int m = 0; m < N_I_V; m++)
						fwrite((const void *) &(trajectory_lookup_table[i][j][k][l][m]),
								sizeof(TrajectoryLookupTable::TrajectoryControlParameters), 1, tlt_f);
	fclose(tlt_f);
}


bool
TrajectoryLookupTable::load_trajectory_lookup_table()
{
	struct stat buffer;

	if (stat("trajectory_lookup_table.bin", &buffer) == 0)
	{
		FILE *tlt_f;

		tlt_f = fopen("trajectory_lookup_table.bin", "r");

		for (int i = 0; i < N_DIST; i++)
			for (int j = 0; j < N_THETA; j++)
				for (int k = 0; k < N_D_YAW; k++)
					for (int l = 0; l < N_I_PHI; l++)
						for (int m = 0; m < N_I_V; m++)
							fread((void *) &(trajectory_lookup_table[i][j][k][l][m]),
									sizeof(TrajectoryLookupTable::TrajectoryControlParameters), 1, tlt_f);
		fclose(tlt_f);

		return (true);
	}
	else
		return (false);
}


int
get_random_integer(int max)
{
	return (rand() % max);
}


float
geometric_progression(int n, float scale_factor, float ratio, int index_of_element_zero)
{
	if (n >= index_of_element_zero)
		return (scale_factor * pow(ratio, n - index_of_element_zero) - scale_factor);
	else
		return (-(scale_factor * pow(ratio, index_of_element_zero - n) - scale_factor));
}


double
linear_progression(int n, double common, int index_of_element_zero)
{
	if (n == index_of_element_zero)
		return (0.0);
	else if (n > index_of_element_zero)
		return (common * (n - index_of_element_zero));
	else
		return (-common * (index_of_element_zero - n));
}


float
get_initial_velocity_by_index(int index)
{
	return (geometric_progression(index, FIRST_I_V, RATIO_I_V, ZERO_I_V_I));
}


float
get_delta_velocity_by_index(int index)
{
	return (geometric_progression(index, FIRST_D_V, RATIO_D_V, ZERO_D_V_I));
}


float
get_i_phi_by_index(int index)
{
	return (geometric_progression(index, FIRST_I_PHI, RATIO_I_PHI, ZERO_I_PHI_I));
}


float
get_k1_by_index(int index)
{
	return (geometric_progression(index, FIRST_K1, RATIO_K1, ZERO_K1_I));
}


float
get_k2_by_index(int index)
{
	return (geometric_progression(index, FIRST_K2, RATIO_K2, ZERO_K2_I));
}


float
get_distance_by_index(int index)
{
    return (geometric_progression(index, FIRST_DIST, RATIO_DIST, ZERO_DIST_I));
}


float
get_theta_by_index(int index)
{
    return (linear_progression(index, COMMON_THETA, ZERO_THETA_I));
}


float
get_d_yaw_by_index(int index)
{
    return (geometric_progression(index, FIRST_D_YAW, RATIO_D_YAW, ZERO_D_YAW_I));
}


TrajectoryLookupTable::TrajectoryDimensions
convert_to_trajectory_dimensions(TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd,
        TrajectoryLookupTable::TrajectoryControlParameters tcp)
{
    TrajectoryLookupTable::TrajectoryDimensions td;

    td.d_yaw = get_d_yaw_by_index(tdd.d_yaw);
    td.dist = get_distance_by_index(tdd.dist);
    td.phi_i = get_i_phi_by_index(tdd.phi_i);
    td.theta = get_theta_by_index(tdd.theta);
    td.v_i = get_initial_velocity_by_index(tdd.v_i);
    td.control_parameters = tcp;

    return (td);
}


int
binary_search_geometric_progression(double value,
		int num_elements, double first, double ratio, int zero_index)
{
	int imin = 0;
	int imax = num_elements - 1;
	while (imin < imax)
	{
		int imid = imin + (imax - imin) / 2;

		if (geometric_progression(imid, first, ratio, zero_index) < value)
			imin = imid + 1;
		else
			imax = imid;
	}
	return (imin);
}


int
binary_search_linear_progression(double value,
		int num_elements, double common, int zero_index)
{
	int imin = 0;
	int imax = num_elements - 1;
	while (imin < imax)
	{
		int imid = imin + (imax - imin) / 2;

		if (linear_progression(imid, common, zero_index) < value)
			imin = imid + 1;
		else
			imax = imid;
	}
	return (imin);
}


TrajectoryLookupTable::TrajectoryDiscreteDimensions
get_discrete_dimensions(TrajectoryLookupTable::TrajectoryDimensions td)
{
	TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd;

	tdd.dist = binary_search_geometric_progression(td.dist, N_DIST, FIRST_DIST, RATIO_DIST, ZERO_DIST_I);
	tdd.theta = binary_search_linear_progression(td.theta, N_THETA, COMMON_THETA, ZERO_THETA_I);
	tdd.d_yaw = binary_search_geometric_progression(td.d_yaw, N_D_YAW, FIRST_D_YAW, RATIO_D_YAW, ZERO_D_YAW_I);
	tdd.phi_i = binary_search_geometric_progression(td.phi_i, N_I_PHI, FIRST_I_PHI, RATIO_I_PHI, ZERO_I_PHI_I);
	tdd.v_i = binary_search_geometric_progression(td.v_i, N_I_V, FIRST_I_V, RATIO_I_V, ZERO_I_V_I);

	return (tdd);
}


TrajectoryLookupTable::TrajectoryControlParameters
generate_trajectory_control_parameters_sample(int k1, int k2, int i_v, int dist)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp;

	tcp.valid = true;

	tcp.k1 = get_k1_by_index(k1);
	tcp.k2 = get_k2_by_index(k2);
	tcp.k3 = 0.0;
	tcp.has_k3 = false;

	tcp.v0 = get_initial_velocity_by_index(i_v);
	double distance = get_distance_by_index(dist);
	tcp.velocity_profile = LINEAR_PROFILE;

    // s = s0 + v0.t + 1/2.a.t^2
    // s0 = 0; v0 = tcp.v0; t = PROFILE_TIME
    tcp.a0 = (2.0 * (distance - tcp.v0 * PROFILE_TIME)) / (PROFILE_TIME * PROFILE_TIME);
    tcp.vf = tcp.v0 + tcp.a0 * PROFILE_TIME;
    tcp.vt = tcp.vf;
    tcp.tf = tcp.tt = tcp.t0 = PROFILE_TIME;
    tcp.af = 0.0;

	return (tcp);
}


TrajectoryLookupTable::TrajectoryControlParameters
generate_trajectory_control_parameters_sample(double k1, double k2, int i_v, int dist)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp;

	tcp.valid = true;

	tcp.k1 = k1;
	tcp.k2 = k2;

	tcp.v0 = get_initial_velocity_by_index(i_v);
	double distance = get_distance_by_index(dist);
	tcp.velocity_profile = LINEAR_PROFILE;

	double time;
	if (distance > 7.0)
		time = PROFILE_TIME;
	else if (distance > 3.5)
		time = PROFILE_TIME / 2.0;
	else
		time = PROFILE_TIME / 2.5;
//	double time = PROFILE_TIME;
    // s = s0 + v0.t + 1/2.a.t^2
	// a = (s - s0 - v0.t) / (t^2.1/2)
    // s = distance; s0 = 0; v0 = tcp.v0; t = time
    tcp.a0 = (distance - tcp.v0 * time) / (time * time * 0.5);
    // v = v0 + a.t
    tcp.vf = tcp.v0 + tcp.a0 * time;
    tcp.vt = tcp.vf;
    tcp.tf = tcp.tt = tcp.t0 = time;
    tcp.af = 0.0;

	return (tcp);
}


TrajectoryLookupTable::TrajectoryControlParameters
generate_random_trajectory_control_parameters_sample(int i_v, int dist)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp;

	tcp.v0 = get_initial_velocity_by_index(i_v);
	float distance = get_distance_by_index(dist);
	if (distance == (tcp.v0 * PROFILE_TIME))
		tcp.velocity_profile = CONSTANT_PROFILE;
	else
		tcp.velocity_profile = LINEAR_PROFILE; // (TrajectoryVelocityProfile) (get_random_integer(NUM_VELOCITY_PROFILES - 2) + 1); // skip CONSTANT_PROFILE

	switch (tcp.velocity_profile)
	{
	case CONSTANT_PROFILE:
		tcp.vf = tcp.vt = tcp.v0;
		tcp.tf = tcp.tt = tcp.t0 = PROFILE_TIME;
		tcp.a0 = 0.0;
		tcp.af = 0.0;
		break;
	case LINEAR_PROFILE:
		// distance = delta_t * (v0 + vt) / 2 -> trapezoidal rule
		tcp.vt = 2.0 * distance / PROFILE_TIME - tcp.v0;
		tcp.vf = tcp.vt;
		tcp.tf = tcp.tt = tcp.t0 = PROFILE_TIME;
		tcp.a0 = (tcp.vf - tcp.v0) / tcp.tf;
		tcp.af = 0.0;
		break;
	case LINEAR_RAMP_PROFILE:
		tcp.t0 = PROFILE_TIME / 2.0;
		tcp.tf = tcp.tt = PROFILE_TIME;
		tcp.vt = (tcp.t0 * tcp.v0 - 2.0 * distance) / (tcp.t0 - 2.0 * tcp.tf);
		tcp.vf = tcp.vt;
		tcp.a0 = (tcp.vf - tcp.v0) / tcp.t0;
		tcp.af = 0.0;
		break;
	case TRAPEZOIDAL_PROFILE:
		tcp.t0 = PROFILE_TIME / 3.0;
		tcp.tt = tcp.t0 * 2.0;
		tcp.tf = PROFILE_TIME;
		do
		{
			tcp.vt = tcp.v0 + get_delta_velocity_by_index(get_random_integer(N_D_V));
			tcp.vf = (-2.0 * distance + tcp.t0 * (tcp.v0 - tcp.vt) + tcp.vt * (tcp.tt + tcp.tf)) / (tcp.tt - tcp.tf);
		} while ((tcp.vf < 0.0) || (tcp.vt < 0.0));
		tcp.a0 = (tcp.vt - tcp.v0) / tcp.t0;
		tcp.af = (tcp.vf - tcp.vt) / (tcp.tf - tcp.tt);
		break;
	}

//	float d0 = ((tcp.v0 + tcp.vt) / 2.0) * tcp.t0;
//	float d1 =  tcp.vt * (tcp.tt - tcp.t0);
//	float d2 = ((tcp.vt + tcp.vf) / 2.0) * (tcp.tf - tcp.tt);
//	float d = d0 + d1 + d2;

	tcp.k1 = get_k1_by_index(get_random_integer(N_K1));
	tcp.k2 = get_k2_by_index(get_random_integer(N_K2));

	tcp.valid = true;

	return (tcp);
}


double
predict_next_pose_step(Robot_State &new_robot_state, const Command &requested_command, double delta_t,
		double &achieved_curvature, const double &desired_curvature, double &max_curvature_change)
{
	Robot_State initial_robot_state = new_robot_state;

	double delta_curvature = fabs(achieved_curvature - desired_curvature);
	double command_curvature_signal = (achieved_curvature < desired_curvature) ? 1.0 : -1.0;

	achieved_curvature = achieved_curvature + command_curvature_signal * fmin(delta_curvature, max_curvature_change);
	new_robot_state.v_and_phi.phi = carmen_get_phi_from_curvature(
			achieved_curvature, initial_robot_state.v_and_phi.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	// Tem que checar se as equacoes que governam esta mudancca de v estao corretas (precisa de um Euler?) e fazer o mesmo no caso do rrt_path_follower.
	double delta_v = fabs(initial_robot_state.v_and_phi.v - requested_command.v);
	double command_v_signal = (initial_robot_state.v_and_phi.v <= requested_command.v) ? 1.0 : -1.0;
	new_robot_state.v_and_phi.v = initial_robot_state.v_and_phi.v + command_v_signal * delta_v;

	double move_x = new_robot_state.v_and_phi.v * delta_t * cos(initial_robot_state.pose.theta);
	double move_y = new_robot_state.v_and_phi.v * delta_t * sin(initial_robot_state.pose.theta);

	new_robot_state.pose.x	   += move_x;
	new_robot_state.pose.y	   += move_y;
	new_robot_state.pose.theta += new_robot_state.v_and_phi.v * delta_t * tan(new_robot_state.v_and_phi.phi) / GlobalState::robot_config.distance_between_front_and_rear_axles;

	return sqrt(move_x * move_x + move_y * move_y);
}


Robot_State
TrajectoryLookupTable::predict_next_pose(Robot_State &robot_state, const Command &requested_command,
		double full_time_interval, double *distance_traveled, double delta_t)
{
	int n = floor(full_time_interval / delta_t);
	double remaining_time = full_time_interval - ((double) n * delta_t);
	Robot_State achieved_robot_state = robot_state; // achieved_robot_state eh computado iterativamente abaixo a partir do estado atual do robo

	double curvature = carmen_get_curvature_from_phi(
			requested_command.phi, requested_command.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	double new_curvature = carmen_get_curvature_from_phi(
			achieved_robot_state.v_and_phi.phi, achieved_robot_state.v_and_phi.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	double max_curvature_change = GlobalState::robot_config.desired_steering_command_rate * delta_t;

	// Euler method
	for (int i = 0; i < n; i++)
	{
		double dist_walked = predict_next_pose_step(achieved_robot_state, requested_command, delta_t,
				new_curvature, curvature, max_curvature_change);

		if (distance_traveled)
			*distance_traveled += dist_walked;

		if (!step_sf.empty())
			step_sf.push_back(*distance_traveled);
	}

	if (remaining_time > 0.0)
	{
		double dist_walked = predict_next_pose_step(achieved_robot_state, requested_command, delta_t,
				new_curvature, curvature, max_curvature_change);

		if (distance_traveled)
			*distance_traveled += dist_walked;

		if (!step_sf.empty())
					step_sf.push_back(*distance_traveled);
	}

	achieved_robot_state.pose.theta = carmen_normalize_theta(achieved_robot_state.pose.theta);
	robot_state = achieved_robot_state;

	return (achieved_robot_state);
}


double
compute_path_via_simulation(Robot_State &robot_state, Command &command,
        vector<carmen_ackerman_path_point_t> &path,
		TrajectoryLookupTable::TrajectoryControlParameters tcp,
		gsl_spline *phi_spline, gsl_interp_accel *acc, double i_phi,
		TrajectoryLookupTable::CarLatencyBuffer car_latency_buffer)
{
    int i = 0;
    double t, last_t;
    double distance_traveled = 0.0;
    double delta_t = 0.15;
    int reduction_factor = 1 + (int)((tcp.tf / delta_t) / 90.0);

    robot_state.pose.x = 0.0;
    robot_state.pose.y = 0.0;
    robot_state.pose.theta = 0.0;
    robot_state.v_and_phi.v = tcp.v0;
    robot_state.v_and_phi.phi = i_phi;
    command.v = tcp.v0;
	command.phi = gsl_spline_eval(phi_spline, 0.0, acc);
	robot_state.v_and_phi = command;
	Robot_State last_robot_state = robot_state;
//	bool is_first_phi = true;
//	bool is_first_v = true;
	for (last_t = t = 0.0; t < tcp.tf; t += delta_t)
	{
		if (t < PHI_LATENCY)
			command.phi = car_latency_buffer.previous_phi[(int) (t / LATENCY_CICLE_TIME)];
		else
			command.phi = gsl_spline_eval(phi_spline, t - PHI_LATENCY, acc);

//		if ((t >= PHI_LATENCY) && is_first_phi)
//		{
//			if (fabs(command.phi - car_latency_buffer.previous_phi[PHI_LATENCY_BUFFER_SIZE - 1]) > carmen_degrees_to_radians(2.0))
//				printf("phi after latency does not match planned phi!!!!!\n");
//
//			is_first_phi = false;
//		}

		if (t < V_LATENCY)
			command.v = car_latency_buffer.previous_v[(int) (t / LATENCY_CICLE_TIME)];
		else
		{
			if (tcp.velocity_profile == LINEAR_PROFILE)
	            command.v += tcp.a0 * delta_t;
	        else if (tcp.velocity_profile == LINEAR_RAMP_PROFILE)
	        {
	            if (t < tcp.t0)
	                command.v += tcp.a0 * delta_t;
	        }
	        else if (tcp.velocity_profile == TRAPEZOIDAL_PROFILE)
	        {
	            if (t < tcp.t0)
	                command.v += tcp.a0 * delta_t;
	            if (t > tcp.tt)
	                command.v += tcp.af * delta_t;
	        }
		}

//		if ((t >= V_LATENCY) && is_first_v)
//		{
//			if (fabs(command.v - car_latency_buffer.previous_v[V_LATENCY_BUFFER_SIZE - 1]) > 0.2)
//				printf("v after latency does not match planned v!!!!!\n");
//
//			is_first_v = false;
//		}

		TrajectoryLookupTable::predict_next_pose(robot_state, command, delta_t,	&distance_traveled, delta_t);
		if ((i % reduction_factor) == 0)
		{
			path.push_back(Util::convert_to_carmen_ackerman_path_point_t(last_robot_state, t + delta_t - last_t));
			last_robot_state = robot_state;
			last_t = t + delta_t;
		}
		i++;
	}
	if ((t - tcp.tf) != delta_t)
	{
		delta_t = delta_t - (t - tcp.tf);
		command.phi = gsl_spline_eval(phi_spline, tcp.tf, acc);
        if (tcp.velocity_profile == LINEAR_PROFILE)
            command.v += tcp.a0 * delta_t;
        else if (tcp.velocity_profile == LINEAR_RAMP_PROFILE)
        {
            if (t < tcp.t0)
                command.v += tcp.a0 * delta_t;
        }
        else if (tcp.velocity_profile == TRAPEZOIDAL_PROFILE)
        {
            if (t < tcp.t0)
                command.v += tcp.a0 * delta_t;
            if (t > tcp.tt)
                command.v += tcp.af * delta_t;
        }
		TrajectoryLookupTable::predict_next_pose(robot_state, command, delta_t, &distance_traveled, delta_t);
		path.push_back(Util::convert_to_carmen_ackerman_path_point_t(last_robot_state, tcp.tf - last_t));
	}

	return (distance_traveled);
}


void
print_phi_profile(gsl_spline *phi_spline, gsl_interp_accel *acc, double total_t, bool display_phi_profile)
{
    if (!display_phi_profile)
        return;

    FILE *path_file = fopen("gnuplot_path2.txt", "w");

    for (double t = 0.0; t < total_t; t += total_t / 100.0)
        fprintf(path_file, "%f %f\n", t, gsl_spline_eval(phi_spline, t, acc));
    fclose(path_file);

    FILE *gnuplot_pipe = popen("gnuplot -persist", "w");
    fprintf(gnuplot_pipe, "plot './gnuplot_path2.txt' using 1:2 with lines\n");
    fflush(gnuplot_pipe);
    pclose(gnuplot_pipe);
    getchar();
    system("pkill gnuplot");
}

void
print_phi_profile_temp(gsl_spline *phi_spline, gsl_interp_accel *acc, double total_t, bool display_phi_profile)
{
	static int iteracao = 1;
    if (!display_phi_profile)
        return;
    char phi_path[20];
    sprintf(phi_path, "phi/%d.txt", iteracao);
    FILE *path_file = fopen("gnu_tests/phi_plot.txt" , "w");

    for (double t = 0.0; t < total_t; t += total_t / 100.0)
        fprintf(path_file, "%f %f\n", t, gsl_spline_eval(phi_spline, t, acc));
    fclose(path_file);
    iteracao++;
}


vector<carmen_ackerman_path_point_t>
simulate_car_from_parameters(TrajectoryLookupTable::TrajectoryDimensions &td,
		TrajectoryLookupTable::TrajectoryControlParameters &tcp, double i_phi,
		TrajectoryLookupTable::CarLatencyBuffer car_latency_buffer,	bool display_phi_profile)
{
	vector<carmen_ackerman_path_point_t> path;
	if (!tcp.valid)
	{
		printf("Warning: invalid TrajectoryControlParameters tcp in simulate_car_from_parameters()\n");
		return (path);
	}

	// Create phi profile

	gsl_interp_accel *acc;
	gsl_spline *phi_spline;
	if (tcp.has_k3)
	{
		//double knots_x[4] = {0.0, tcp.tf / 3.0, 2.0 * (tcp.tf / 3.0), tcp.tf};
		double knots_x[4] = {0.0, tcp.tf / 4.0, tcp.tf / 2.0, tcp.tf};
		double knots_y[4] = {i_phi, tcp.k1, tcp.k2, tcp.k3};
		acc = gsl_interp_accel_alloc();
		const gsl_interp_type *type = gsl_interp_cspline;
		phi_spline = gsl_spline_alloc(type, 4);
		gsl_spline_init(phi_spline, knots_x, knots_y, 4);
	}
	else
	{
		double knots_x[3] = {0.0, tcp.tf / 2.0, tcp.tf};
		double knots_y[3] = {i_phi, tcp.k1, tcp.k2};
		acc = gsl_interp_accel_alloc();
		const gsl_interp_type *type = gsl_interp_cspline;
		phi_spline = gsl_spline_alloc(type, 3);
		gsl_spline_init(phi_spline, knots_x, knots_y, 3);
	}
	print_phi_profile_temp(phi_spline, acc, tcp.tf, display_phi_profile);

	Command command;
    Robot_State robot_state;

    double distance_traveled = compute_path_via_simulation(robot_state, command, path, tcp, phi_spline, acc, i_phi, car_latency_buffer);

	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);
	td.dist = sqrt(robot_state.pose.x * robot_state.pose.x + robot_state.pose.y * robot_state.pose.y);
	td.theta = atan2(robot_state.pose.y, robot_state.pose.x);
	td.d_yaw = robot_state.pose.theta;
	td.phi_i = i_phi;
	td.v_i = tcp.v0;
	tcp.vf = command.v;
	tcp.sf = distance_traveled;
	td.control_parameters = tcp;

	return (path);
}


void
print_path(vector<carmen_ackerman_path_point_t> path)
{
	FILE *path_file = fopen("gnuplot_path.txt", "a");
	int i = 0;
	for (std::vector<carmen_ackerman_path_point_t>::iterator it = path.begin(); it != path.end(); ++it)
	{
		if ((i % 2) == 0)
			fprintf(path_file, "%f %f %f %f\n", it->x, it->y, 1.0 * cos(it->theta), 1.0 * sin(it->theta));
		i++;
	}
	fclose(path_file);
}

void
print_lane(vector<carmen_ackerman_path_point_t> path, FILE *path_file)
{

//	int i = 0;
	for (std::vector<carmen_ackerman_path_point_t>::iterator it = path.begin(); it != path.end(); ++it)
	{
//		if ((i % 2) == 0)
			fprintf(path_file, "%f %f %f %f\n", it->x, it->y, 1.0 * cos(it->theta), 1.0 * sin(it->theta));
//		i++;
	}
}

TrajectoryLookupTable::TrajectoryDimensions
compute_trajectory_dimensions(TrajectoryLookupTable::TrajectoryControlParameters &tcp, int i_phi,
        vector<carmen_ackerman_path_point_t> &path, bool print)
{
	double d_i_phi = get_i_phi_by_index(i_phi);
    TrajectoryLookupTable::TrajectoryDimensions td;
    path = simulate_car_from_parameters(td, tcp, d_i_phi, g_car_latency_buffer, print);
    if (tcp.valid && print)
        print_path(path);

	return (td);
}


TrajectoryLookupTable::TrajectoryControlParameters
fill_in_tcp(const gsl_vector *x, ObjectiveFunctionParams *params)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp;

	if (x->size == 4)
	{
		tcp.has_k3 = true;

		tcp.k1 = gsl_vector_get(x, 0);
		tcp.k2 = gsl_vector_get(x, 1);
		tcp.k3 = gsl_vector_get(x, 2);
		tcp.tf = gsl_vector_get(x, 3);
	}
	else
	{
		tcp.k1 = gsl_vector_get(x, 0);
		tcp.k2 = gsl_vector_get(x, 1);
		tcp.tf = gsl_vector_get(x, 2);

		tcp.k3 = 0.0;
		tcp.has_k3 = true;
	}
	tcp.v0 = params->target_td->v_i;
	tcp.a0 = params->suitable_acceleration;
	tcp.af = params->tcp_seed->af;
	tcp.t0 = params->tcp_seed->t0;
    tcp.tt = params->tcp_seed->tt;
    tcp.vt = params->tcp_seed->vt;
    tcp.vf = params->tcp_seed->vf;
    tcp.sf = params->tcp_seed->sf;

	tcp.velocity_profile = (TrajectoryVelocityProfile) ((int) params->tcp_seed->velocity_profile);
	tcp.valid = true;

	return (tcp);
}


void
move_path_to_current_robot_pose(vector<carmen_ackerman_path_point_t> &path, Pose *localizer_pose)
{
    for (std::vector<carmen_ackerman_path_point_t>::iterator it = path.begin(); it != path.end(); ++it)
    {
        double x = localizer_pose->x + it->x * cos(localizer_pose->theta) - it->y * sin(localizer_pose->theta);
        double y = localizer_pose->y + it->x * sin(localizer_pose->theta) + it->y * cos(localizer_pose->theta);
        it->x = x;
        it->y = y;
        it->theta += localizer_pose->theta;
    }
}


double
compute_abstacles_cost(vector<carmen_ackerman_path_point_t> path)
{
    double max_obstacles_cost = 0.0;

    move_path_to_current_robot_pose(path, GlobalState::localizer_pose);
    for (unsigned int i = 0; i < path.size(); i++)
    {
        carmen_point_t robot_pose = {path[i].x, path[i].y, path[i].theta};

        double current_cost = carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot_border(
             &robot_pose, &GlobalState::cost_map,
             GlobalState::robot_config.length, GlobalState::robot_config.width,
             GlobalState::robot_config.distance_between_rear_car_and_rear_wheels);
        if (current_cost > max_obstacles_cost)
            max_obstacles_cost = current_cost;
    }

    // std::cout << "max_obstacles_cost = " << max_obstacles_cost << std::endl;
    return (max_obstacles_cost * 0.08);
}


double
dist(carmen_ackerman_path_point_t v, carmen_ackerman_path_point_t w)
{
	return sqrt((carmen_square(v.x - w.x) + carmen_square(v.y - w.y)));
}

double
dist2(carmen_ackerman_path_point_t v, carmen_ackerman_path_point_t w)
{
	return (carmen_square(v.x - w.x) + carmen_square(v.y - w.y));
}

carmen_ackerman_path_point_t
get_the_point_nearest_to_the_trajectory(int *point_in_trajectory_is,
		carmen_ackerman_path_point_t current_robot_position,
		carmen_ackerman_path_point_t waypoint,
		carmen_ackerman_path_point_t center_of_the_car_front_axel)
{

#define	WITHIN_THE_TRAJECTORY		0
#define	CURRENT_ROBOT_POSITION		1
#define	BEFORE_CURRENT_ROBOT_POSITION	2
#define	BEYOND_WAYPOINT			3

	// Return minimum distance between line segment vw and point p
	// http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
	carmen_ackerman_path_point_t v, w, p;
	double l2, t;

	p.x = center_of_the_car_front_axel.x;
	p.y = center_of_the_car_front_axel.y;
	p.v = 0;
	p.phi = 0;
	p.theta = 0;

	v.x = current_robot_position.x;
	v.y = current_robot_position.y;
	v.v = 0;
	v.phi = 0;
	v.theta = 0;

	w.x = waypoint.x;
	w.y = waypoint.y;
	w.v = 0;
	w.phi = 0;
	w.theta = 0;

	l2 = dist2(v, w); // i.e. |w-v|^2 // NAO TROQUE POR carmen_ackerman_traj_distance2(&v, &w) pois nao sei se ee a mesma coisa.
	if (l2 < 0.1)	  // v ~== w case // @@@ Alberto: Checar isso
	{
		*point_in_trajectory_is = CURRENT_ROBOT_POSITION;
		return (v);
	}

	// Consider the line extending the segment, parameterized as v + t (w - v).
	// We find the projection of point p onto the line.
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;

	if (t < 0.0) 	// p beyond the v end of the segment
	{
		*point_in_trajectory_is = BEFORE_CURRENT_ROBOT_POSITION;
		return (v);
	}
	if (t > 1.0)	// p beyond the w end of the segment
	{
		*point_in_trajectory_is = BEYOND_WAYPOINT;
		return (w);
	}

	// Projection falls on the segment
	p.x = v.x + t * (w.x - v.x);
	p.y = v.y + t * (w.y - v.y);
	*point_in_trajectory_is = WITHIN_THE_TRAJECTORY;

	return (p);
}

double
get_distance_between_point_to_line(carmen_ackerman_path_point_t current_robot_position,
		carmen_ackerman_path_point_t waypoint,
		carmen_ackerman_path_point_t center_of_the_car_front_axel)
{
	//https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	double delta_x = waypoint.x - current_robot_position.x;
	double delta_y = waypoint.y - current_robot_position.y;
	double d = sqrt(delta_x * delta_x + delta_y * delta_y);
	double x2y1 =  waypoint.x * current_robot_position.y;
	double y2x1 =  waypoint.y * current_robot_position.x;

	if (d < 0.0000001)
		return dist(waypoint, center_of_the_car_front_axel);

	return abs(delta_y * center_of_the_car_front_axel.x - delta_x * center_of_the_car_front_axel.y + x2y1 - y2x1) / d;

}

void
get_points(vector<carmen_ackerman_path_point_t> &detailed_goal_list, carmen_ackerman_path_point_t &path_point, int &index_p1, int &index_p2)
{
	double max1 = DBL_MAX;
	double max2 = DBL_MAX;
	unsigned int idx1 = 0 ,  idx2 = 0;


	for (unsigned int i = 0; i < detailed_goal_list.size(); i++)
	{
		double d = dist(detailed_goal_list.at(i), path_point);
		if (max1 > d)
		{
			max1 = d;
			idx1 = i;
		}
	}
	for (unsigned int i = 0; i < detailed_goal_list.size(); i++)
	{
		double d = dist(detailed_goal_list.at(i), path_point);
		if (max2 > d && i != idx1)
		{
			max2 = d;
			idx2 = i;
		}
	}
	index_p1 = idx1;
	index_p2 = idx2;
}

//Sigmoid para dar peso para distancia
double inline
sigmoid(double x, double z)
{
	return (1/(1+exp(-x * 0.9 + z)));
}


double inline
gaussian(double x, double z)
{
	return exp((-0.3/2)*((x - z) * (x - z)));
}


//double
//compute_interest_dist_new(vector<carmen_ackerman_path_point_t> &detailed_goal_list, vector<carmen_ackerman_path_point_t> &path, vector<unsigned int> &nearest_path_point)
//{
////	https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
//
//	double middle_of_lane = lane_step_sf[(lane_step_sf.size()/2)];
//	double distance = 0.0;
//	double total_distance = 0.0;
//	int p1 = 0;
//	int p2 = 0;
//	for (unsigned int i = 0; i < path.size(); i++)
//	{
//		get_points(detailed_goal_list, path.at(i), p1, p2);
//		distance = get_distance_between_point_to_line(detailed_goal_list.at(p1), detailed_goal_list.at(p2), path.at(i));
//		distance = distance * sigmoid(lane_step_sf[i], (middle_of_lane-2));
//		total_distance += (distance*distance);
//	}
//
//	return (total_distance);
//}


/*TODO
 * Seria necessario o primeiro ponto do path (x=0 e y=0) entrar no total_distance?
 * */
double
compute_interest_dist(vector<carmen_ackerman_path_point_t> &detailed_goal_list, vector<carmen_ackerman_path_point_t> &path, vector<unsigned int> &nearest_path_point)
{
	double distance = 0.0;
	double total_distance = 0.0;
//	double middle_of_lane = lane_step_sf[(lane_step_sf.size()/2)];
	double total_points = 0.0;
	for (unsigned int i = 0; i < detailed_goal_list.size(); i += 3)
	{
		if (nearest_path_point.at(i) < path.size())
		{
			distance = dist(path.at(nearest_path_point.at(i)), detailed_goal_list.at(i));
//			distance = distance * gaussian(lane_step_sf[i], middle_of_lane-2);
		}
		else
			distance = 0.0;
		total_distance += distance;//(distance*distance);
		total_points += 1.0;

//	printf("Path x: %lf y: %lf \n", path.at(i).x, path.back().y);
	}
	return (total_distance / total_points);
}


double
compute_reference_path(ObjectiveFunctionParams *param, vector<carmen_ackerman_path_point_t> &path)
{
	double distance = 0.0;
	double min_dist;
	int index = 0;
	param->nearest_path_point.clear();
	param->path_size = path.size();
	for (unsigned int i = 0; i < param->detailed_goal_list.size(); i++)
	{
		// consider the first point as the nearest one
		min_dist = dist(param->detailed_goal_list.at(i), path.at(0));

		for (unsigned int j = index; j < path.size(); j++)
		{
			distance = dist(param->detailed_goal_list.at(i), path.at(j));

			if (distance < min_dist)
			{
				min_dist = distance;
				index = j;
			}
		}
		param->nearest_path_point.push_back(index);
	}
	return (compute_interest_dist(param->detailed_goal_list, path, param->nearest_path_point));

}


double
compute_proximity_to_obstacles(vector<carmen_ackerman_path_point_t> path)
{
	double proximity_to_obstacles = 0.0;
	double min_dist = 2.5 / 2.0; // metade da largura do carro
    int k = 1;
    for (unsigned int i = 0; i < path.size(); i += 4)
    {
    	// Move path point to map coordinates
    	carmen_ackerman_path_point_t path_point_in_map_coords;
    	double x_gpos = GlobalState::localizer_pose->x - GlobalState::cost_map.config.x_origin;
    	double y_gpos = GlobalState::localizer_pose->y - GlobalState::cost_map.config.y_origin;
    	path_point_in_map_coords.x = x_gpos + path[i].x * cos(GlobalState::localizer_pose->theta) - path[i].y * sin(GlobalState::localizer_pose->theta);
    	path_point_in_map_coords.y = y_gpos + path[i].x * sin(GlobalState::localizer_pose->theta) + path[i].y * cos(GlobalState::localizer_pose->theta);

	    // Search for nearest neighbors in the map
	    vector<occupied_cell> returned_occupied_cells;
	    occupied_cell sought = occupied_cell(path_point_in_map_coords.x, path_point_in_map_coords.y);
	    // knn search
	    GlobalState::obstacles_rtree.query(bgi::nearest(sought, k), std::back_inserter(returned_occupied_cells));

	    carmen_ackerman_path_point_t nearest_obstacle;
	    nearest_obstacle.x = returned_occupied_cells[0].get<0>();
	    nearest_obstacle.y = returned_occupied_cells[0].get<1>();

	    double distance = dist(path_point_in_map_coords, nearest_obstacle);
	    double delta = distance - min_dist;
	    if (delta < 0.0)
	    	proximity_to_obstacles += delta * delta;
    }
    return (proximity_to_obstacles);
}


double
my_f(const gsl_vector *x, void *params)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(x, my_params);
	TrajectoryLookupTable::TrajectoryDimensions td;

	if (tcp.tf < 0.2) // o tempo nao pode ser pequeno demais
		tcp.tf = 0.2;

	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->phi_i, g_car_latency_buffer, false);
	//TCP_SEED nao eh modificado pelo CG?
	my_params->tcp_seed->vf = tcp.vf;
	my_params->tcp_seed->sf = tcp.sf;

	double result = sqrt((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
			(carmen_normalize_theta(td.theta) - my_params->target_td->theta) * (carmen_normalize_theta(td.theta) - my_params->target_td->theta) / (my_params->theta_by_index * 0.2) +
			(carmen_normalize_theta(td.d_yaw) - my_params->target_td->d_yaw) * (carmen_normalize_theta(td.d_yaw) - my_params->target_td->d_yaw) / (my_params->d_yaw_by_index * 0.2));

	return (result);
}


/* The gradient of f, df = (df/dx, df/dy). */
void
my_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	double f_x = my_f(v, params);

	double h = 0.00001;

	gsl_vector *x_h;
	x_h = gsl_vector_alloc(3);

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0) + h);
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	double f_x_h = my_f(x_h, params);
	double d_f_x_h = (f_x_h - f_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1) + h);
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	double f_y_h = my_f(x_h, params);
	double d_f_y_h = (f_y_h - f_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2) + h);
	double f_z_h = my_f(x_h, params);
	double d_f_z_h = (f_z_h - f_x) / h;

	gsl_vector_set(df, 0, d_f_x_h);
	gsl_vector_set(df, 1, d_f_y_h);
	gsl_vector_set(df, 2, d_f_z_h);

	gsl_vector_free(x_h);
}


/* Compute both f and df together. */
void
my_fdf(const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f(x, params);
	my_df(x, params, df);
}



//Compute cost function to optimize lane
double
my_g(const gsl_vector *x, void *params)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(x, my_params);
	TrajectoryLookupTable::TrajectoryDimensions td;

	if (tcp.tf < 0.2) // o tempo nao pode ser pequeno demais
		tcp.tf = 0.2;

	//esse vetor eh a distancia entre cada ponto do path o ultimo ponto do vetor eh o tcp.sf
	step_sf.clear();
	step_sf.push_back(0.0);

	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->phi_i, g_car_latency_buffer, false);

	double total_interest_dist = 0.0;
	if (use_lane)
	{
		if (path.size() != my_params->path_size)
			total_interest_dist = compute_reference_path(my_params, path);
		else
			total_interest_dist = compute_interest_dist(my_params->detailed_goal_list, path, my_params->nearest_path_point);
	}

	double proximity_to_obstacles = 0.0;
	if (use_obstacles && !GlobalState::obstacles_rtree.empty())
		proximity_to_obstacles = compute_proximity_to_obstacles(path);

	my_params->tcp_seed->vf = tcp.vf;
	my_params->tcp_seed->sf = tcp.sf;
	my_params->lane_sf = total_interest_dist;

//
//	FILE *path_file_dist = fopen("gnu_tests/gnuplot_path_dist.txt", "w");
//	print_lane(path,path_file_dist);
//	fclose(path_file_dist);

//	printf("Distancia: %lf \n" , total_interest_dist);
//	double goal_dist = dist(path.back(), my_params->detailed_goal_list.back());
//	goal_dist *= goal_dist;
//	double d_yaw = carmen_normalize_theta((path.back().phi - my_params->detailed_goal_list.back().phi));
//	double dist_2 = total_interest_dist * total_interest_dist;

//	double result = (goal_dist*0.8) + (total_interest_dist * 0.1); //goal_dist nao tava ao quadrado
//	double result = (sqrt(((goal_dist*0.08) + (d_yaw*0.2))*1) + (total_interest_dist * 0.05));
//	double result = (goal_dist*0.1) + (d_yaw*0.001) + (total_interest_dist*0.01);
//	double result = (goal_dist * 0.5) + (total_interest_dist * 0.05) + (d_yaw);
	double result = sqrt(
				1.0 * (td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
				5.0 * (carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / my_params->theta_by_index +
				5.0 * (carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / my_params->d_yaw_by_index +
				1.5 * (total_interest_dist * total_interest_dist) +
				0.2 * proximity_to_obstacles); // já é quandrática
//	printf("Goal dist: %lf \t sem peso: %lf \n", (goal_dist*0.1),  (goal_dist));
//	printf("total_interest: %lf \t sem peso %lf \n", (total_interest_dist * 0.1), (total_interest_dist));

	return (result);

}


/* The gradient of f, df = (df/dx, df/dy). */
void
my_dg(const gsl_vector *v, void *params, gsl_vector *df)
{
	double g_x = my_g(v, params);

	double h = 0.00001;//<<< 0.00001

	gsl_vector *x_h;
	x_h = gsl_vector_alloc(4);

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0) + h);
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3));
	double g_x_h = my_g(x_h, params);
	double d_g_x_h = (g_x_h - g_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1) + h);
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3));
	double g_y_h = my_g(x_h, params);
	double d_g_y_h = (g_y_h - g_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2) + h);
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3));
	double g_z_h = my_g(x_h, params);
	double d_g_z_h = (g_z_h - g_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3) + h);
	double g_w_h = my_g(x_h, params);
	double d_g_w_h = (g_w_h - g_x) / h;

	gsl_vector_set(df, 0, d_g_x_h);
	gsl_vector_set(df, 1, d_g_y_h);
	gsl_vector_set(df, 2, d_g_z_h);
	gsl_vector_set(df, 3, d_g_w_h);

	gsl_vector_free(x_h);
}


/* Compute both g and df together. for while df equal to dg */
void
my_gdf(const gsl_vector *x, void *params, double *g, gsl_vector *dg)
{
	*g = my_g(x, params);
	my_dg(x, params, dg);
}


double
compute_suitable_acceleration(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v)
{
	// (i) S = Vo*t + 1/2*a*t^2
	// (ii) dS/dt = Vo + a*t
	// dS/dt = 0 => máximo ou mínimo de S => 0 = Vo + a*t; a*t = -Vo; (iii) a = -Vo/t; (iv) t = -Vo/a
	// Se "a" é negativa, dS/dt = 0 é um máximo de S
	// Logo, como S = target_td.dist, "a" e "t" tem que ser tais em (iii) e (iv) que permitam que
	// target_td.dist seja alcançada.
	//
	// O valor de maxS pode ser computado substituindo (iv) em (i):
	// maxS = Vo*-Vo/a + 1/2*a*(-Vo/a)^2 = -Vo^2/a + 1/2*Vo^2/a = -1/2*Vo^2/a

	double a = (target_v - target_td.v_i) / tcp_seed.tf;

	if (a >= 0.0)
		return (a);

	if ((-0.5 * (target_td.v_i * target_td.v_i) / a) > target_td.dist * 1.1)
		return (a);
	else
	{
		while ((-0.5 * (target_td.v_i * target_td.v_i) / a) <= target_td.dist * 1.1)
			a *= 0.95;
		return (a);
	}
}


void
add_points_to_goal_list_interval(carmen_ackerman_path_point_t p1, carmen_ackerman_path_point_t p2, vector<carmen_ackerman_path_point_t> &detailed_goal_list)
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

	for(i = 0; i < num_points; i++)
	{
		new_point.x = p1.x + i * delta_x;
		new_point.y = p1.y + i * delta_y;
		new_point.theta = p1.theta + i * delta_theta;

		detailed_goal_list.push_back(new_point);
	}
}


void
copy_starting_nearest_point_of_zero(vector<carmen_ackerman_path_point_t> &detailed_goal_list, vector<carmen_ackerman_path_point_t> &temp_detail, double *lane_sf)
{
	detailed_goal_list.clear();
	lane_step_sf.clear();
	lane_step_sf.push_back(0.0);

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
			{
				detailed_goal_list.push_back(temp_detail.at(j));
				if (1 < detailed_goal_list.size())
				{
					*lane_sf += dist(detailed_goal_list.at(k-1), detailed_goal_list.at(k));
					lane_step_sf.push_back(*lane_sf);
				}
//				printf("Lane x: %lf y: %lf \n", detailed_goal_list.back().x, detailed_goal_list.back().y);
			}
			break;
		}
	}
}


bool
build_detailed_goal_list(vector<carmen_ackerman_path_point_t> *lane_in_local_pose, vector<carmen_ackerman_path_point_t> &detailed_goal_list, double *lane_sf)
{
	if (lane_in_local_pose->size() > 7)
	{
		vector<carmen_ackerman_path_point_t> temp_detail;
		for (int i = 0; i < (lane_in_local_pose->size() - 1); i++)
		{
			add_points_to_goal_list_interval(lane_in_local_pose->at(i), lane_in_local_pose->at(i+1), temp_detail);
		}
		//add last point
		temp_detail.push_back(lane_in_local_pose->back());
		//mantendo primeiro ponto mais proximo de 0
		copy_starting_nearest_point_of_zero(detailed_goal_list, temp_detail, lane_sf);
	}
	else
	{
		printf(KGRN "+++++++++++++ ERRO MENSAGEM DA LANE POSES !!!!\n" RESET);
		return (false);
	}
	return (true);
}


//TODO Calcular valor de fator para quando o carro estiver muito distante da lane
double
car_lane_distance_factor(double car_lane_distance, double lane_sf_2)
{
    return (2*car_lane_distance)/(1 + exp(-car_lane_distance*0.9 + lane_sf_2));
}


TrajectoryLookupTable::TrajectoryControlParameters
optimized_lane_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, vector<carmen_ackerman_path_point_t> *lane_in_local_pose,
		ObjectiveFunctionParams params, vector<double> &cost_results)
{

	// A f(x) muntidimensional que queremos minimizar é:
	//   f(x) = ||(car_simulator(x) + lane(x) < max_dist_lane)||
	// e as dimensões de f(x) são (dist, theta, d_yaw, phi_i, v_i, v_f)
	// O resultado ideal é zero, isto é, a saida de car_simulator deve ser igual a distancia max da lane.
	// As variáveis que serão modificadas pela minimização são:
	//   k1, k2 e tf
	// E, durante a minimização:
	//   v_0, v_i e phi_i são constantes
	//   v_0 = v_i
	//   vt, a0, af, t0, tt e sf sao dependentes das demais segundo o TrajectoryVelocityProfile

	double lane_sf = 0.0;
	if (!build_detailed_goal_list(lane_in_local_pose, params.detailed_goal_list, &lane_sf))
	{
		cost_results.push_back(1.5);
		return (tcp_seed);
	}
//	printf("Lane x: %lf y: %lf \n", params.detailed_goal_list.back().x, params.detailed_goal_list.back().y);

	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(target_td, tcp_seed,target_td.phi_i, g_car_latency_buffer, false);
//	double total_distance = compute_reference_path(&params, path);

	//TODO jah posso testar aqui se o path ja esta otimizado para a lane
//	if (total_distance < valor_aceitavel)
//		return (tcp_seed);

//	printf("Path x: %lf y: %lf \n", path.back().x, path.back().y);

//	FILE *lane_file = fopen("gnu_tests/gnuplot_lane.txt", "w");
//	print_lane(params.detailed_goal_list, lane_file);
//	fclose(lane_file);
//	char path_name[20];
//	sprintf(path_name, "path/%d.txt", 0);
//	FILE *path_file = fopen("gnu_tests/gnuplot_traj.txt", "w");
//	print_lane(path,path_file);
//	fclose(path_file);
//	getchar();


	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;

	//	double par[17] = {0 target_td.v_i, 1 target_td.phi_i, 2 - target_td.dist, 3 - target_td.theta, 4 - target_td.d_yaw,
	//			5 - suitable_acceleration, 6 - tcp_seed.af, 7 - tcp_seed.t0, 8 - tcp_seed.tt, 9 - tcp_seed.vt, 10 - target_v,
	//			11 - (double) ((int) tcp_seed.velocity_profile), 12 - tcp_seed.vf, 13 - tcp_seed.sf,
	//			14 - fabs(get_distance_by_index(N_DIST-1)),
	//			15 - fabs(get_theta_by_index(N_THETA-1)), 16 - fabs(get_d_yaw_by_index(N_D_YAW-1))}

	params.distance_by_index = fabs(get_distance_by_index(N_DIST-1));
	params.theta_by_index = fabs(get_theta_by_index(N_THETA-1));
	params.d_yaw_by_index = fabs(get_d_yaw_by_index(N_D_YAW-1));
	params.target_td = &target_td;
	params.tcp_seed = &tcp_seed;
	params.target_v = target_v;
	params.lane_sf = 0.0;

	gsl_vector *x;
	gsl_multimin_function_fdf my_func;

	my_func.n = 4;
	my_func.f = my_g;
	my_func.df = my_dg;
	my_func.fdf = my_gdf;
	my_func.params = &params;

	double knots_x[3] = {0.0, tcp_seed.tf / 2.0, tcp_seed.tf};
	double knots_y[3] = {target_td.phi_i, tcp_seed.k1, tcp_seed.k2};
	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	gsl_spline *phi_spline = gsl_spline_alloc(type, 3);
	gsl_spline_init(phi_spline, knots_x, knots_y, 3);

//	print_phi_profile_temp(phi_spline, acc, tcp.tf, display_phi_profile);

	/* Starting point, x */
	x = gsl_vector_alloc(4);
//	gsl_vector_set(x, 0, gsl_spline_eval(phi_spline, tcp_seed.tf / 3.0, acc));
//	gsl_vector_set(x, 1, gsl_spline_eval(phi_spline, 2.0 * (tcp_seed.tf / 3.0), acc));
	gsl_vector_set(x, 0, gsl_spline_eval(phi_spline, tcp_seed.tf / 4.0, acc));
	gsl_vector_set(x, 1, gsl_spline_eval(phi_spline, tcp_seed.tf / 2.0, acc));
	gsl_vector_set(x, 2, gsl_spline_eval(phi_spline, tcp_seed.tf, acc));
	gsl_vector_set(x, 3, tcp_seed.tf);

	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);

	T = gsl_multimin_fdfminimizer_conjugate_fr;
	s = gsl_multimin_fdfminimizer_alloc(T, 4);

	// int gsl_multimin_fdfminimizer_set (gsl_multimin_fdfminimizer * s, gsl_multimin_function_fdf * fdf, const gsl_vector * x, double step_size, double tol)
	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.01, 0.001);

	size_t iter = 0;
	int status;
//	double actual_car_to_lane_distance = dist(params.detailed_goal_list[0], path[0]);
//	double MAX_LANE_DIST = 0.50 + car_lane_distance_factor(actual_car_to_lane_distance, (lane_sf/2));
//	printf("Max_lane_dist: %lf \n", MAX_LANE_DIST);
	do
	{
		iter++;

		status = gsl_multimin_fdfminimizer_iterate(s);

		if (status == GSL_ENOPROG) // minimizer is unable to improve on its current estimate, either due to numerical difficulty or a genuine local minimum
		{
//			printf("@@@@@@@@@@@@@@ status = %d\n", status);
			break;
		}

		// int gsl_multimin_test_gradient (const gsl_vector * g, double epsabs)
		// |g| < epsabs	} while ((s->f > MAX_LANE_DIST) && (status == GSL_CONTINUE) && (iter < 300)); //alterado de 0.005

		status = gsl_multimin_test_gradient(s->gradient, 0.16); // esta funcao retorna GSL_CONTINUE ou zero

		//	--Debug with GNUPLOT

//		TrajectoryLookupTable::TrajectoryControlParameters tcp_temp = fill_in_tcp(s->x, &params);
//		char path_name[20];
//		sprintf(path_name, "path/%lu.txt", iter);
//		FILE *path_file = fopen("gnu_tests/gnuplot_traj.txt", "w");
//		print_lane(simulate_car_from_parameters(target_td, tcp_temp, target_td.phi_i, g_car_latency_buffer, true), path_file);
//		fclose(path_file);
//		printf("Estou na: %lu iteracao, sf: %lf  \n", iter, s->f);
//		getchar();
		//	--

	} while (/*(s->f > MAX_LANE_DIST) &&*/ (status == GSL_CONTINUE) && (iter < 300)); //alterado de 0.005

//	printf("Parei em: %lu iteracoes, sf: %lf  \n", iter, s->f);
//	getchar();

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);

//	//TODO Verificar esse teste para a lane
	if ((tcp.tf < 0.2)/* || (s->f > 0.75)*/) // too short plan or bad minimum (s->f should be close to zero)
		tcp.valid = false;

	cost_results.push_back(params.lane_sf);
	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (tcp);
}


// TODO optimizer
TrajectoryLookupTable::TrajectoryControlParameters
get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, ObjectiveFunctionParams &params)
{
	// A f(x) muntidimensional que queremos minimizar é:
	//   f(x) = ||(car_simulator(x) - target_td, vf - target_v)||
	// e as dimensões de f(x) são (dist, theta, d_yaw, phi_i, v_i, v_f)
	// O resultado ideal é zero, isto é, a saida de car_simulator deve ser igual a target_td e vf = target_v.
	// As variáveis que serão modificadas pela minimização são:
	//   k1, k2 e tf
	// E, durante a minimização:
	//   v_0, v_i e phi_i são constantes
	//   v_0 = v_i
	//   vt, a0, af, t0, tt e sf sao dependentes das demais segundo o TrajectoryVelocityProfile

	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;

	double suitable_acceleration = compute_suitable_acceleration(tcp_seed, target_td, target_v);

	//	double par[17] = {0 target_td.v_i, 1 target_td.phi_i, 2 - target_td.dist, 3 - target_td.theta, 4 - target_td.d_yaw,
	//			5 - suitable_acceleration, 6 - tcp_seed.af, 7 - tcp_seed.t0, 8 - tcp_seed.tt, 9 - tcp_seed.vt, 10 - target_v,
	//			11 - (double) ((int) tcp_seed.velocity_profile), 12 - tcp_seed.vf, 13 - tcp_seed.sf,
	//			14 - fabs(get_distance_by_index(N_DIST-1)),
	//			15 - fabs(get_theta_by_index(N_THETA-1)), 16 - fabs(get_d_yaw_by_index(N_D_YAW-1))}

	params.distance_by_index = fabs(get_distance_by_index(N_DIST-1));
	params.theta_by_index = fabs(get_theta_by_index(N_THETA-1));
	params.d_yaw_by_index = fabs(get_d_yaw_by_index(N_D_YAW-1));
	params.suitable_acceleration = suitable_acceleration;
	params.target_td = &target_td;
	params.tcp_seed = &tcp_seed;
	params.target_v = target_v;
	params.lane_sf = 0.0;

	
	gsl_vector *x;
	gsl_multimin_function_fdf my_func;

	my_func.n = 3;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = &params;

	/* Starting point, x */
	x = gsl_vector_alloc(3);
	gsl_vector_set(x, 0, tcp_seed.k1);
	gsl_vector_set(x, 1, tcp_seed.k2);
	gsl_vector_set(x, 2, tcp_seed.tf);

	T = gsl_multimin_fdfminimizer_conjugate_fr;
	s = gsl_multimin_fdfminimizer_alloc(T, 3);

	// int gsl_multimin_fdfminimizer_set (gsl_multimin_fdfminimizer * s, gsl_multimin_function_fdf * fdf, const gsl_vector * x, double step_size, double tol)
	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.0001, 0.001);

	size_t iter = 0;
	int status;
	do
	{
		iter++;

		status = gsl_multimin_fdfminimizer_iterate(s);


		if (status == GSL_ENOPROG) // minimizer is unable to improve on its current estimate, either due to numerical difficulty or a genuine local minimum
		{
			//printf("@@@@@@@@@@@@@@ status = %d\n", status);
			break;
		}

		// int gsl_multimin_test_gradient (const gsl_vector * g, double epsabs)
		// |g| < epsabs
		status = gsl_multimin_test_gradient(s->gradient, 0.16); // esta funcao retorna GSL_CONTINUE ou zero

	} while ((s->f > 0.005) && (status == GSL_CONTINUE) && (iter < 300)); //alterado de 0.005

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);

	if ((tcp.tf < 0.2) || (s->f > 0.05)) // too short plan or bad minimum (s->f should be close to zero) mudei de 0.05 para outro
		tcp.valid = false;

	if (target_td.dist < 3.0 && tcp.valid == false) // para debugar
		tcp.valid = false;


	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

//	if (tcp.valid)
//	{
//		print_path(simulate_car_from_parameters(target_td, tcp, target_td.phi_i, g_car_latency_buffer, false));
//		print_path(params.detailed_goal_list);
//		FILE *gnuplot_pipe = popen("gnuplot -persist", "w");
//		fprintf(gnuplot_pipe, "set xrange [-15:45]\nset yrange [-15:15]\n"
//				"plot './gnuplot_path.txt' using 1:2:3:4 w vec size  0.3, 10 filled\n");
//		fflush(gnuplot_pipe);
//		pclose(gnuplot_pipe);
//		getchar();
//		system("pkill gnuplot");
//	}
//	printf("Iteracoes: %lu \n", iter);
	return (tcp);
}

TrajectoryLookupTable::TrajectoryControlParameters
get_complete_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, vector<carmen_ackerman_path_point_t> *lane_in_local_pose, vector<double> &cost_results)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp_complete;
	ObjectiveFunctionParams params;

	tcp_complete = get_optimized_trajectory_control_parameters(tcp_seed, target_td, target_v, params);
	if (tcp_complete.valid)
		tcp_complete = optimized_lane_trajectory_control_parameters(tcp_complete, target_td, target_v, lane_in_local_pose, params, cost_results);

	return (tcp_complete);

}


//TrajectoryLookupTable::TrajectoryControlParameters
//get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd,
//        TrajectoryLookupTable::TrajectoryDimensions td)
//{
//    TrajectoryLookupTable::TrajectoryControlParameters tcp = td.control_parameters;
//
//    return (tcp);
//}


TrajectoryLookupTable::TrajectoryControlParameters
get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
        TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd, double target_v)
{
	ObjectiveFunctionParams params;

	TrajectoryLookupTable::TrajectoryDimensions target_td = convert_to_trajectory_dimensions(tdd, tcp_seed);
    TrajectoryLookupTable::TrajectoryControlParameters tcp = get_optimized_trajectory_control_parameters(tcp_seed, target_td, target_v, params);

    return (tcp);
}



TrajectoryLookupTable::TrajectoryControlParameters
get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
        TrajectoryLookupTable::TrajectoryDiscreteDimensions &tdd, double target_v, vector<carmen_ackerman_path_point_t> optimized_path)
{
	ObjectiveFunctionParams params;

    TrajectoryLookupTable::TrajectoryDimensions target_td = convert_to_trajectory_dimensions(tdd, tcp_seed);
    TrajectoryLookupTable::TrajectoryControlParameters tcp = get_optimized_trajectory_control_parameters(tcp_seed, target_td, target_v, params);
    if (tcp.valid)
    {
		TrajectoryLookupTable::TrajectoryDimensions td;
		optimized_path = simulate_car_from_parameters(td, tcp, tdd.phi_i, g_car_latency_buffer, false);
		tdd = get_discrete_dimensions(td);
    }

    return (tcp);
}


bool
same_tcp(TrajectoryLookupTable::TrajectoryControlParameters tcp1, TrajectoryLookupTable::TrajectoryControlParameters tcp2)
{
    bool result = true;

    if (tcp1.a0 != tcp2.a0)
    {
        //std::cout << "tcp1.a0 = " << tcp1.a0 << "  tcp2.a0 = " << tcp2.a0 << std::endl;
        result = false;
    }
    if (tcp1.af != tcp2.af)
    {
        //std::cout << "tcp1.af = " << tcp1.af << "  tcp2.af = " << tcp2.af << std::endl;
        result = false;
    }
    if (tcp1.k1 != tcp2.k1)
    {
        //std::cout << "tcp1.k1 = " << tcp1.k1 << "  tcp2.k1 = " << tcp2.k1 << std::endl;
        result = false;
    }
    if (tcp1.k2 != tcp2.k2)
    {
        //std::cout << "tcp1.k2 = " << tcp1.k2 << "  tcp2.k2 = " << tcp2.k2 << std::endl;
        result = false;
    }
    if (tcp1.sf != tcp2.sf)
    {
        //std::cout << "tcp1.sf = " << tcp1.sf << "  tcp2.sf = " << tcp2.sf << std::endl;
        result = false;
    }
    if (tcp1.t0 != tcp2.t0)
    {
        //std::cout << "tcp1.t0 = " << tcp1.t0 << "  tcp2.t0 = " << tcp2.t0 << std::endl;
        result = false;
    }
    if (tcp1.tt != tcp2.tt)
    {
        //std::cout << "tcp1.tt = " << tcp1.tt << "  tcp2.tt = " << tcp2.tt << std::endl;
        result = false;
    }
    if (tcp1.tf != tcp2.tf)
    {
        //std::cout << "tcp1.tf = " << tcp1.tf << "  tcp2.tf = " << tcp2.tf << std::endl;
        result = false;
    }
    if (tcp1.v0 != tcp2.v0)
    {
       // std::cout << "tcp1.v0 = " << tcp1.v0 << "  tcp2.v0 = " << tcp2.v0 << std::endl;
        result = false;
    }
    if (tcp1.vf != tcp2.vf)
    {
        //std::cout << "tcp1.vf = " << tcp1.vf << "  tcp2.vf = " << tcp2.vf << std::endl;
        result = false;
    }
    if (tcp1.vt != tcp2.vt)
    {
        //std::cout << "tcp1.vt = " << tcp1.vt << "  tcp2.vt = " << tcp2.vt << std::endl;
        result = false;
    }
    if (tcp1.velocity_profile != tcp2.velocity_profile)
    {
        //std::cout << "tcp1.velocity_profile = " << tcp1.velocity_profile << "  tcp2.velocity_profile = " << tcp2.velocity_profile << std::endl;
        result = false;
    }
    if (tcp1.valid != tcp2.valid)
    {
        //std::cout << "tcp1.valid = " << tcp1.valid << "  tcp2.valid = " << tcp2.valid << std::endl;
        result = false;
    }
    return (result);
}


bool
compare_td(TrajectoryLookupTable::TrajectoryDimensions td1, TrajectoryLookupTable::TrajectoryDimensions td2)
{
    bool result = true;

    if (td1.d_yaw != td2.d_yaw)
    {
        std::cout << "td1.d_yaw = " << td1.d_yaw << "  td2.d_yaw = " << td2.d_yaw << std::endl;
        result = false;
    }
    if (td1.dist != td2.dist)
    {
        std::cout << "td1.dist = " << td1.dist << "  td2.dist = " << td2.dist << std::endl;
        result = false;
    }
    if (td1.phi_i != td2.phi_i)
    {
        std::cout << "td1.phi_i = " << td1.phi_i << "  td2.phi_i = " << td2.phi_i << std::endl;
        result = false;
    }
    if (td1.theta != td2.theta)
    {
        std::cout << "td1.theta = " << td1.theta << "  td2.theta = " << td2.theta << std::endl;
        result = false;
    }
    if (td1.v_i != td2.v_i)
    {
        std::cout << "td1.v_i = " << td1.v_i << "  td2.v_i = " << td2.v_i << std::endl;
        result = false;
    }
    return (result);
}


//void
//fill_in_trajectory_lookup_table_old()
//{
//	TrajectoryLookupTable::TrajectoryControlParameters tcp;
//	TrajectoryLookupTable::TrajectoryDimensions td;
//	TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd;
//
////	for (int i = 0; i < 10; i++)
//	for (int dist = N_DIST - 1; dist >= 0; dist--)
////	for (int dist = 14; dist < 15; dist++)
//	{
//		printf("dist = %d\n\n\n", dist);
//		for (int theta = 0; theta < N_THETA; theta++)
////		for (int theta = 17; theta < 18; theta++)
//		{
//			printf("theta = %d\n", theta);
//			for (int d_yaw = 0; d_yaw < N_D_YAW; d_yaw++)
////			for (int d_yaw = 7; d_yaw < 8; d_yaw++)
//				for (int i_phi = 0; i_phi < N_I_PHI; i_phi++)
//				{
//					//printf("i_phi = %d\n", i_phi);
//
//					for (int i_v = 0; i_v < N_I_V; i_v++)
////					for (int i_v = 5; i_v < 6; i_v++)
//					{
//						tcp = generate_random_trajectory_control_parameters_sample(i_v, dist);
//						vector<carmen_ackerman_path_point_t> path;
//                        td = compute_trajectory_dimensions(tcp, i_phi, path, false);
////                        td = compute_trajectory_dimensions(tcp, i_phi, path, true);
////						FILE *gnuplot_pipe = popen("gnuplot -persist", "w");
////						fprintf(gnuplot_pipe, "plot './gnuplot_path.txt' using 1:2:3:4 w vec size  0.3, 10 filled\n");
////						fflush(gnuplot_pipe);
////						pclose(gnuplot_pipe);
////						getchar();
////						system("pkill gnuplot");
//
//						tdd = get_discrete_dimensions(td);
//						TrajectoryLookupTable::TrajectoryDimensions ntd = convert_to_trajectory_dimensions(tdd, tcp);
//						std::cout << "td.phi_i = " << td.phi_i << std::endl;
//						compare_td(td, ntd);
////						getchar();
//						if (has_valid_discretization(tdd) && !trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i].valid)
//						{
//							trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i] =
//                                    get_optimized_trajectory_control_parameters(tcp, tdd, tcp.vf);
//                                    //get_optimized_trajectory_control_parameters(tdd, td);
//							//printf("[tdd.dist] %02d, [tdd.theta] %02d, [tdd.d_yaw] %02d, [tdd.phi_i] %02d, [tdd.v_i] %02d\n", tdd.dist, tdd.theta, tdd.d_yaw, tdd.phi_i, tdd.v_i);
//	                        vector<carmen_ackerman_path_point_t> path;
//	                        compute_trajectory_dimensions(trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i], i_phi, path, true);
//
//	                        if (trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i].valid)
//	                        {
//                                FILE *gnuplot_pipe = popen("gnuplot -persist", "w");
//                                fprintf(gnuplot_pipe, "plot './gnuplot_path.txt' using 1:2:3:4 w vec size  0.3, 10 filled\n");
//                                fflush(gnuplot_pipe);
//                                pclose(gnuplot_pipe);
//                                getchar();
//                                system("pkill gnuplot");
//	                        }
//                            if (!same_tcp(trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i], tcp))
//                                std::cout << "tcp differents!!!!!!!!!!!!!!!!!!" << std::endl;
//						}
//						//else
//							//printf("[tdd.dist] %02d, [tdd.theta] %02d, [tdd.d_yaw] %02d, [tdd.phi_i] %02d, [tdd.v_i] %02d, @@@@@@@@@@@@@@@@\n", tdd.dist, tdd.theta, tdd.d_yaw, tdd.phi_i, tdd.v_i);
//					}
//				}
//		}
//	}
//}


bool
path_has_loop(vector<carmen_ackerman_path_point_t> path)
{
    bool loop[24] = { false };
    for (unsigned int i = 0; i < path.size(); i++)
    {
        int index = (int) (floor(24.0 * ((carmen_normalize_theta(path[i].theta) + M_PI) / (2.0 * M_PI))));
        if (index > 23)
            index = 23;
        if (index < 0)
            index = 0;
        loop[index] = true;
    }

    bool has_loop = true;
    for (int i = 0; i < 24; i++)
        if (!loop[i])
            has_loop = false;

    return (has_loop);
}


bool
path_has_loop(double dist, double sf)
{
	if (sf > (M_PI * dist * 1.5)) // se sf for maior que meio arco com diametro dist mais um pouco (1.5) tem loop
		return (true);
	return (false);
}


void
fill_in_trajectory_lookup_table_new_old()
{
	#pragma omp parallel for
    for (int dist = N_DIST - 1; dist >= 0; dist--)
    {
        printf("dist = %d\n\n", dist);

        //for (int i_phi = ZERO_I_PHI_I; i_phi < N_I_PHI; i_phi++)
        for (int i_phi = 0; i_phi < N_I_PHI; i_phi++)
        {
            printf("dist = %d, i_phi = %d\n", dist, i_phi);
            //for (int k1 = ZERO_K1_I; k1 < N_K1; k1++)
            for (int k1 = 0; k1 < N_K1; k1++)
                //for (int k2 = ZERO_K2_I; k2 < N_K2; k2++)
                for (int k2 = 0; k2 < N_K2; k2++)
                    for (int i_v = 0; i_v < N_I_V; i_v++)
					    // Trocar d_v acima por dist e mudar generate_trajectory_control_parameters_sample()
					    // abaixo para tratar de acordo usando o CONSTANT e LINEAR velocity profiles.
					    // Tratar apenas deslocamentos para frente. Checar se os limites do carro estao
					    // sendo obedecidos (ate o phi dentro dos splines?)
					    // Otimizar e checar se eh valida a trajetoria usando as facilidades de
					    // visualizacao ja implementadas.
					{
                    	TrajectoryLookupTable::TrajectoryControlParameters tcp = generate_trajectory_control_parameters_sample(k1, k2, i_v, dist);
						vector<carmen_ackerman_path_point_t> path;
						TrajectoryLookupTable::TrajectoryDimensions td = compute_trajectory_dimensions(tcp, i_phi, path, false);
//                        td = compute_trajectory_dimensions(tcp, i_phi, path, true);
//                        FILE *gnuplot_pipe = popen("gnuplot -persist", "w");
//                        fprintf(gnuplot_pipe, "plot './gnuplot_path.txt' using 1:2:3:4 w vec size  0.3, 10 filled\n");
//                        fflush(gnuplot_pipe);
//                        pclose(gnuplot_pipe);
//                        getchar();
//                        system("pkill gnuplot");

						TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd = get_discrete_dimensions(td);
//                        TrajectoryLookupTable::TrajectoryDimensions ntd = convert_to_trajectory_dimensions(tdd, tcp);
//                        std::cout << "td.phi_i = " << td.phi_i << std::endl;
//                        compare_td(td, ntd);
						if (has_valid_discretization(tdd))
						{
	                        TrajectoryLookupTable::TrajectoryControlParameters ntcp = get_optimized_trajectory_control_parameters(tcp, tdd, tcp.vf);
	                        if (!path_has_loop(get_distance_by_index(tdd.dist), ntcp.sf))
	                        {
	                            TrajectoryLookupTable::TrajectoryControlParameters ptcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
	                            if (!ptcp.valid || (ntcp.valid && (ntcp.sf < ptcp.sf)))
	                            {
	                                trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i] = ntcp;

//	                                vector<carmen_ackerman_path_point_t> path;
//	                                compute_trajectory_dimensions(trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i], i_phi, path, true);
//                                    FILE *gnuplot_pipe = popen("gnuplot -persist", "w");
//                                    fprintf(gnuplot_pipe, "plot './gnuplot_path.txt' using 1:2:3:4 w vec size  0.3, 10 filled\n");
//                                    fflush(gnuplot_pipe);
//                                    pclose(gnuplot_pipe);
//                                    getchar();
//                                    system("pkill gnuplot");
//                                    if (!same_tcp(trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i], tcp))
//                                        std::cout << "tcp differents!!!!!!!!!!!!!!!!!!" << std::endl;
	                            }
	                        }
						}
					}
        }
    }
}


void
fill_in_trajectory_lookup_table()
{
	#pragma omp parallel for
    for (int dist = N_DIST - 1; dist >= 0; dist--)
    {
        printf("dist = %d\n\n", dist);
        fflush(stdout);
        for (int i_phi = 0; i_phi < N_I_PHI; i_phi++)
        {
        	double entries = 0.0;
        	double valids = 0.0;

        	//i_phi = N_I_PHI/2;
//			for (int k1 = 0; k1 < N_K1; k1++)
//			{
//				for (int k2 = 0; k2 < N_K2; k2++)
//				{
            double delta_k1 = (get_k1_by_index(N_K1-1) - get_k1_by_index(0)) / 60.0;
            for (double k1 = get_k1_by_index(0); k1 < get_k1_by_index(N_K1-1); k1 += delta_k1)
            {
                double delta_k2 = (get_k2_by_index(N_K2-1) - get_k2_by_index(0)) / 60.0;
            	for (double k2 = get_k2_by_index(0); k2 < get_k2_by_index(N_K2-1); k2 += delta_k2)
            	{
                    for (int i_v = 0; i_v < N_I_V; i_v++)
					    // Checar se os limites do carro estao
					    // sendo obedecidos (ate o phi dentro dos splines?)
					    // Otimizar e checar se eh valida a trajetoria usando as facilidades de
					    // visualizacao ja implementadas.
					{
                    	//i_v = 7;
                    	TrajectoryLookupTable::TrajectoryControlParameters tcp = generate_trajectory_control_parameters_sample(k1, k2, i_v, dist);
						vector<carmen_ackerman_path_point_t> path;
						TrajectoryLookupTable::TrajectoryDimensions td = compute_trajectory_dimensions(tcp, i_phi, path, false);
//                        td = compute_trajectory_dimensions(tcp, i_phi, path, true);
//                        FILE *gnuplot_pipe = popen("gnuplot -persist", "w");
//                        fprintf(gnuplot_pipe, "plot './gnuplot_path.txt' using 1:2:3:4 w vec size  0.3, 10 filled\n");
//                        fflush(gnuplot_pipe);
//                        pclose(gnuplot_pipe);
//                        getchar();
//                        system("pkill gnuplot");

						TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd = get_discrete_dimensions(td);
//                        TrajectoryLookupTable::TrajectoryDimensions ntd = convert_to_trajectory_dimensions(tdd, tcp);
//                        std::cout << "td.phi_i = " << td.phi_i << std::endl;
//                        compare_td(td, ntd);
		            	entries += 1.0;
						if (has_valid_discretization(tdd))
						{
							// vector<carmen_ackerman_path_point_t> optimized_path;
	                        TrajectoryLookupTable::TrajectoryControlParameters ntcp = get_optimized_trajectory_control_parameters(tcp, tdd, tcp.vf);//, optimized_path);
//	                        if (!ntcp.valid)
//	                        	printf("dist = %lf, i_phi = %lf, k1 = %lf, k2 = %lf, i_v = %lf\n",
//	                        			get_distance_by_index(dist), get_i_phi_by_index(i_phi), k1, k2, get_initial_velocity_by_index(i_v));
	                        if (ntcp.valid && has_valid_discretization(tdd) && !path_has_loop(get_distance_by_index(tdd.dist), ntcp.sf))
	                        {
	                            TrajectoryLookupTable::TrajectoryControlParameters ptcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
	                            if (!ptcp.valid || (ntcp.sf < ptcp.sf))
	                            {
	                                trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i] = ntcp;
	                                valids += 1.0;

//	                                vector<carmen_ackerman_path_point_t> path;
//	                                compute_trajectory_dimensions(trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i], i_phi, path, true);
//                                    FILE *gnuplot_pipe = popen("gnuplot -persist", "w");
//                                    fprintf(gnuplot_pipe, "plot './gnuplot_path.txt' using 1:2:3:4 w vec size  0.3, 10 filled\n");
//                                    fflush(gnuplot_pipe);
//                                    pclose(gnuplot_pipe);
//                                    getchar();
//                                    system("pkill gnuplot");
//                                    if (!same_tcp(trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i], tcp))
//                                        std::cout << "tcp differents!!!!!!!!!!!!!!!!!!" << std::endl;
	                            }
	                        }
						}
					}
            	}
            }
            printf("dist = %d, i_phi = %d, valids = %5.0lf, entries = %5.0lf, %%valids = %2.2lf\n",
            		dist, i_phi, valids, entries, 100.0 * (valids / entries));
            fflush(stdout);
        }
    }
}


void
TrajectoryLookupTable::update_lookup_table_entries()
{
	int num_new_entries = 1;

	while (num_new_entries != 0)
	{
		num_new_entries = 0;
		#pragma omp parallel for
		for (int i = 0; i < N_DIST; i++)
		{
			for (int j = 0; j < N_THETA; j++)
			{
				for (int k = 0; k < N_D_YAW; k++)
				{
					printf("num_new_entries = %d, dist = %d, theta = %d\n", num_new_entries, i, j);
					fflush(stdout);
					for (int l = 0; l < N_I_PHI; l++)
					{
						for (int m = 0; m < N_I_V; m++)
						{
							if (!trajectory_lookup_table[i][j][k][l][m].valid)
							{
								TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd;
								tdd.dist = i; tdd.theta = j; tdd.d_yaw = k; tdd.phi_i = l; tdd.v_i = m;
								TrajectoryLookupTable::TrajectoryControlParameters tcp = search_lookup_table(tdd);
								if (tcp.valid)
								{
									TrajectoryLookupTable::TrajectoryControlParameters ntcp = get_optimized_trajectory_control_parameters(tcp, tdd, tcp.vf);
									if (ntcp.valid && !path_has_loop(get_distance_by_index(tdd.dist), ntcp.sf))
									{
										trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i] = ntcp;
										num_new_entries++;
									}
								}
							}
						}
					}
				}
			}
		}
	}
	save_trajectory_lookup_table();
}


void
TrajectoryLookupTable::build_trajectory_lookup_table()
{
	init_trajectory_lookup_table();
	fill_in_trajectory_lookup_table();
	save_trajectory_lookup_table();
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

    for (i = 0; i < path.size(); i += 2)
    	if ((i + 1) < path.size())
    		path[i].time += path[i + 1].time;
    for (i = 1; i < path.size(); i += 2)
        path.erase(path.begin() + i);
}


void
apply_system_latencies(vector<carmen_ackerman_path_point_t> &path)
{
	unsigned int i, j;

    for (i = 0; i < path.size(); i++)
    {
    	j = i;
    	for (double lat = 0.0; lat < 0.5; j++)
    	{
    		if (j >= path.size())
    			break;
    		lat += path[j].time;
    	}
        path[i].phi = path[j].phi;
    }
    while (i < path.size() - 1)
    	path.pop_back();

    for (i = 0; i < path.size(); i++)
    {
    	j = i;
    	for (double lat = 0.0; lat < 1.0; j++)
    	{
    		if (j >= path.size())
    			break;
    		lat += path[j].time;
    	}
        path[i].v = path[j].v;
    }
    while (i < path.size() - 1)
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
path_has_collision(vector<carmen_ackerman_path_point_t> path)
{
	carmen_point_t pose;
	carmen_robot_ackerman_config_t car_config;

	car_config.distance_between_rear_car_and_rear_wheels =
			GlobalState::robot_config.distance_between_rear_car_and_rear_wheels;
	car_config.length = GlobalState::robot_config.length;
	car_config.width = GlobalState::robot_config.width;

	for (unsigned int j = 0; j < path.size(); j++)
	{
		pose.x = path[j].x;
		pose.y = path[j].y;
		pose.theta = path[j].theta;
		if (obstacle_avoider_pose_hit_obstacle(pose, &GlobalState::cost_map, &car_config))
		{
			//printf("---------- HIT OBSTACLE!!!!\n");
			return (true);
		}
	}
	return (false);
}


void
put_shorter_path_in_front(vector<vector<carmen_ackerman_path_point_t> > &paths, int shorter_path)
{
	if (paths.size() > 1)
	{
		vector<carmen_ackerman_path_point_t> shoter_path;
		shoter_path = paths[shorter_path];
		paths.erase(paths.begin() + shorter_path);
		paths.insert(paths.begin(), shoter_path);
	}
}


void
add_plan_segment_to_car_latency_buffer(vector<carmen_ackerman_path_point_t> path)
{
	double t = 0.0;

	unsigned int j;
	for (int i = j = 0; i < (int) (MAX_PLANNING_TIME / LATENCY_CICLE_TIME); i++)
	{
		g_car_latency_buffer.previous_v[i + V_LATENCY_BUFFER_SIZE] = path[j].v;
		g_car_latency_buffer.previous_phi[i + PHI_LATENCY_BUFFER_SIZE] = path[j].phi;
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
	if (g_car_latency_buffer.timestamp != 0.0)
	{
		int last_delay = (GlobalState::localizer_pose_timestamp - g_car_latency_buffer.timestamp) / LATENCY_CICLE_TIME;
		for (int i = 0; i < V_LATENCY_BUFFER_TOTAL_SIZE - last_delay; i++)
			g_car_latency_buffer.previous_v[i + last_delay] = g_car_latency_buffer.previous_v[i];
		for (int i = 0; i < last_delay; i++)
			g_car_latency_buffer.previous_v[i] = c.v;

		for (int i = 0; i < PHI_LATENCY_BUFFER_TOTAL_SIZE - last_delay; i++)
			g_car_latency_buffer.previous_phi[i + last_delay] = g_car_latency_buffer.previous_phi[i];
		for (int i = 0; i < last_delay; i++)
			g_car_latency_buffer.previous_phi[i] = c.phi;
	}

	g_car_latency_buffer.timestamp = GlobalState::localizer_pose_timestamp;
}


//todo
void
compute_paths(const vector<Command> &lastOdometryVector, vector<Pose> &goalPoseVector, double target_v,
		Pose *localizer_pose, vector<vector<carmen_ackerman_path_point_t> > &paths,
		carmen_rddf_road_profile_message *goal_list_message)
{
	vector<carmen_ackerman_path_point_t> lane_in_local_pose;
	static TrajectoryLookupTable::TrajectoryControlParameters previous_good_tcp;
	TrajectoryLookupTable::TrajectoryControlParameters best_otcp;
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

//	FILE *problems;
//	problems = fopen("problems.txt", "a");

	int path_order = 0;
	int shorter_path = 0;
	double shorter_path_size = 1000.0;
//	double min_cost = 1000.0;
	vector<double> cost_results;

	int errors = 0;
	for (unsigned int i = 0; i < lastOdometryVector.size(); i++)
	{
		for (unsigned int j = 0; j < goalPoseVector.size(); j++)
		{
			TrajectoryLookupTable::TrajectoryControlParameters tcp;

			if (j == 0)
				use_lane = true;
			else
				use_lane = false;

			TrajectoryLookupTable::TrajectoryDimensions td = get_trajectory_dimensions_from_robot_state(localizer_pose, lastOdometryVector[i], &goalPoseVector[j]);
			if (!previous_good_tcp.valid)
			{
				TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd = get_discrete_dimensions(td);
				if (!has_valid_discretization(tdd))
				{
					printf("Invalid discretization!!!!\n");
//					write_tdd_to_file(problems, tdd, "Invalid discretization: ");
					continue;
				}
				tcp = search_lookup_table(tdd);
				if (!tcp.valid)
				{
					printf(KMAG "@@@@@@@@@@@ Could not find a valid entry in the table!!!!\n\033[0m");
//					write_tdd_to_file(problems, tdd, "Could not find: ");
					continue;
				}
			}
			else
				tcp = previous_good_tcp;
			TrajectoryLookupTable::TrajectoryControlParameters otcp;

			otcp = get_complete_optimized_trajectory_control_parameters(tcp, td, target_v, &lane_in_local_pose, cost_results);
			//otcp = get_optimized_trajectory_control_parameters(tcp, td, target_v, &lane_in_local_pose);

			if (otcp.valid)
			{
				vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(td, otcp, td.phi_i, g_car_latency_buffer, false);
				if (path_has_loop(td.dist, otcp.sf))
				{
					printf(KRED "+++++++++++++ Path had loop...\n" RESET);
//					write_tdd_to_file(problems, tdd, "Path had loop: ");
					continue;
				}

				move_path_to_current_robot_pose(path, localizer_pose);
//				apply_system_delay(path);
//				apply_system_latencies(path);

				if (path_has_collision(path))
					continue;

				if (otcp.sf < shorter_path_size)
				{
					shorter_path_size = otcp.sf;
					shorter_path = path_order;
					best_otcp = otcp;
					path_order++;
				}
				filter_path(path);

				paths.push_back(path);
//				if (i == 0 && j == 0)
					break;
			}
			else
			{
//				// para debug: dist: 3, theta: 10, phi_i: 7, v_i: 3, d_yaw: 10
//				if (tdd.dist == 3 && tdd.theta == 10 && tdd.phi_i == 7 && tdd.v_i == 3 && tdd.d_yaw == 10)
//					errors++;
//				write_tdd_to_file(problems, tdd, "Could NOT optimize: ");
				printf(KYEL "+++++++++++++ Could NOT optimize %d !!!!\n" RESET, errors);
			}
		}

		if (paths.size() > 0) // If could find a good path, break. Otherwise, try swerve
			break;
	}
	if (shorter_path > 0)
		put_shorter_path_in_front(paths, shorter_path);

	if (paths.size() > 0)
	{
		previous_good_tcp = best_otcp;
		last_timestamp = carmen_get_time();
	}
	else if ((carmen_get_time() - last_timestamp) > 0.06)
		best_otcp.valid = false;

//		add_plan_segment_to_car_latency_buffer(path[0]);

//	fclose(problems);
}


vector<vector<carmen_ackerman_path_point_t>>
TrajectoryLookupTable::compute_path_to_goal(Pose *localizer_pose, Pose *goal_pose, Command last_odometry,
        double target_v, carmen_rddf_road_profile_message *goal_list_message)
{

	vector<vector<carmen_ackerman_path_point_t>> path;
    vector<Command> lastOdometryVector;
    vector<Pose> goalPoseVector;

    double i_time = carmen_get_time();

	vector<int> magicSignals = {0, 1, -1, 2, -2, 3, -3,  4, -4,  5, -5};
	// @@@ Tranformar os dois loops abaixo em uma funcao -> compute_alternative_path_options()
	for (int i = 0; i < 5; i++)
	{
		Command newOdometry = last_odometry;
		newOdometry.phi +=  0.15 * (double) magicSignals[i]; //(0.5 / (newOdometry.v + 1))
		lastOdometryVector.push_back(newOdometry);
	}

	for (int i = 0; i < 5; i++)
	{
		//printf("Goal x: %lf Goal y: %lf \n",goal_pose->x, goal_pose->y);
		Pose newPose = *goal_pose;
		newPose.x += 0.3 * (double) magicSignals[i] * cos(carmen_normalize_theta((goal_pose->theta) - carmen_degrees_to_radians(90.0)));
		newPose.y += 0.3 * (double) magicSignals[i] * sin(carmen_normalize_theta((goal_pose->theta) - carmen_degrees_to_radians(90.0)));
		goalPoseVector.push_back(newPose);
	}

	compute_paths(lastOdometryVector, goalPoseVector, target_v, localizer_pose, path, goal_list_message);
	printf("%ld plano(s), tempo de planejamento = %lf\n", path.size(), carmen_get_time() - i_time);
	fflush(stdout);

	return (path);
}

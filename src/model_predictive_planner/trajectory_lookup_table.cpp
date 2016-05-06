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
	double lane_sf;
};

#define SYSTEM_DELAY 0.7

TrajectoryLookupTable::TrajectoryControlParameters trajectory_lookup_table[N_DIST][N_THETA][N_D_YAW][N_I_PHI][N_I_V];

double g_last_lane_timestamp = 0.0;


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
	}

	if (remaining_time > 0.0)
	{
		double dist_walked = predict_next_pose_step(achieved_robot_state, requested_command, delta_t,
				new_curvature, curvature, max_curvature_change);

		if (distance_traveled)
			*distance_traveled += dist_walked;
	}

	achieved_robot_state.pose.theta = carmen_normalize_theta(achieved_robot_state.pose.theta);
	robot_state = achieved_robot_state;

	return (achieved_robot_state);
}


double
compute_path_via_simulation(Robot_State &robot_state, Command &command,
        vector<carmen_ackerman_path_point_t> &path,
		TrajectoryLookupTable::TrajectoryControlParameters tcp,
		gsl_spline *phi_spline, gsl_interp_accel *acc, double i_phi)
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
	for (last_t = t = 0.0; t < tcp.tf; t += delta_t)
	{
		command.phi = gsl_spline_eval(phi_spline, t, acc);
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


vector<carmen_ackerman_path_point_t>
simulate_car_from_parameters(TrajectoryLookupTable::TrajectoryDimensions &td,
		TrajectoryLookupTable::TrajectoryControlParameters &tcp, double i_phi,
		bool display_phi_profile = false)
{
	vector<carmen_ackerman_path_point_t> path;
	if (!tcp.valid)
	{
		printf("Warning: invalid TrajectoryControlParameters tcp in simulate_car_from_parameters()\n");
		return (path);
	}

	// Create phi profile
	double knots_x[3] = {0.0, tcp.tf / 2.0, tcp.tf};
	double knots_y[3] = {i_phi, tcp.k1, tcp.k2};
	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	gsl_spline *phi_spline = gsl_spline_alloc(type, 3);
	gsl_spline_init(phi_spline, knots_x, knots_y, 3);

	print_phi_profile(phi_spline, acc, tcp.tf, display_phi_profile);

	Command command;
    Robot_State robot_state;
	double distance_traveled = compute_path_via_simulation(robot_state, command, path, tcp, phi_spline, acc, i_phi);

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


TrajectoryLookupTable::TrajectoryDimensions
compute_trajectory_dimensions(TrajectoryLookupTable::TrajectoryControlParameters &tcp, int i_phi,
        vector<carmen_ackerman_path_point_t> &path, bool print)
{
	double d_i_phi = get_i_phi_by_index(i_phi);
    TrajectoryLookupTable::TrajectoryDimensions td;
    path = simulate_car_from_parameters(td, tcp, d_i_phi, print);
    if (tcp.valid && print)
        print_path(path);

	return (td);
}


TrajectoryLookupTable::TrajectoryControlParameters
fill_in_tcp(const gsl_vector *x, ObjectiveFunctionParams *params)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp;

//	double par[12] = {target_td.v_i, target_td.phi_i, target_td.dist, target_td.theta, target_td.d_yaw,
//		tcp_seed.a0, tcp_seed.af, tcp_seed.t0, tcp_seed.tt, tcp_seed.vt, target_v,
//		(int) tcp_seed.velocity_profile};

	tcp.k1 = gsl_vector_get(x, 0);
	tcp.k2 = gsl_vector_get(x, 1);
	tcp.tf = gsl_vector_get(x, 2);

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
move_path_to_current_robot_pose(vector<carmen_ackerman_path_point_t> &path, Pose *localize_pose)
{
    for (std::vector<carmen_ackerman_path_point_t>::iterator it = path.begin(); it != path.end(); ++it)
    {
        double x = localize_pose->x + it->x * cos(localize_pose->theta) - it->y * sin(localize_pose->theta);
        double y = localize_pose->y + it->x * sin(localize_pose->theta) + it->y * cos(localize_pose->theta);
        it->x = x;
        it->y = y;
        it->theta += localize_pose->theta;
    }
}


double
compute_abstacles_cost(vector<carmen_ackerman_path_point_t> path)
{
    double max_obstacles_cost = 0.0;

    move_path_to_current_robot_pose(path, GlobalState::localize_pose);
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

/*TODO
 * Seria necessario o primeiro ponto do path (x=0 e y=0) entrar no total_distance?
 * */
double
compute_interest_dist(vector<carmen_ackerman_path_point_t> &detailed_goal_list, vector<carmen_ackerman_path_point_t> &path, double lane_sf)
{
	/* Distancia total de interesse
	 * distancia entre cada ponto da lane = tamanho da lane / distancia percorrida na lane (nao a distancia entre pontos, distancia completa do caminho)
	 * fator = distancia percorridaa na lane / distancia percorrida no path
	 * referencia do path na lane = fator * distancia percorrida atual (path[i-1] , path[i])
	 * index na lane = referencia do path na lane / distancia entre cada ponto da lane
	 * ponto exato do path em relacao a lane =  get_the_point_nearest_to_the_trajectory(...,index,index+1,path[i])
	 * retorna distancia(path[i], ponto exato do path em relacao a lane)
	 *
	 * */
	/*TODO - Verificar porque tcp.sf != calculo do sf do path pelos pontos
	 * o tcp_sf esta vindo com um valor superior ao do calculado pelo intervalo de cada ponto no path
	 * apenas testando antes de verificar o problema real
	 * */

	//Lane esta vindo com apenas 1 ponto ou zerada, neste caso nao sera considerada a lane ate uma nova chegar
	if(detailed_goal_list.size() < 2)
		return 0.0;

	double path_sf2 = 0.0;
	for(unsigned int k = 1; k < path.size(); k++)
	{
		path_sf2 += dist(path[k-1], path[k]);
	}

	double fator = lane_sf/path_sf2;
	double lane_detail = (detailed_goal_list.size()-1) / lane_sf;
	double distance_travelled = 0.0;

	double distance = 0.0;
	int index = 0;
	double distance_path_factor = 0.0;
	double total_distance = 0.0;

#ifdef DEBUG_LANE
	printf("----------------Inicio do for-----------------------\n");
	printf("Fator: %lf: \t Lane_sf: %lf \t path_sf: %lf \t Tamanho_Lane: %d \t Tamanho_Path: %d \n", fator, lane_sf, path_sf2, detailed_goal_list.size(), path.size());
#endif

	//	Distancia do primeiro ponto do path(0.0, 0.0) eh necessaria?
	//	distance = dist(detailed_goal_list.at(0), path[0]);
	//	total_distance += distance;

	for(unsigned int i = 1; i < path.size(); i++)
	{
		distance_travelled += dist(path[i-1], path[i]);
		distance_path_factor = fator * distance_travelled;
		index = (int) (distance_path_factor * lane_detail);

//		int info;
//		int index2 = index;
//		if(index < (detailed_goal_list.size()-1))
//			index2++;
//		carmen_ackerman_path_point_t point_path_lane = get_the_point_nearest_to_the_trajectory(&info, detailed_goal_list.at(index), detailed_goal_list.at(index2), path[i]);
//		printf("Ponto na reta: x: %lf y: %lf \n", point_path_lane.x, point_path_lane.y);

		distance = dist(path[i], detailed_goal_list.at(index));
		total_distance += distance;

#ifdef DEBUG_LANE
		printf("lane size no for do dist: %lu \n", detailed_goal_list.size());
		printf("\n Indice: %d \t Indice_Path: %d \n", index, i);
		printf("\n distance_travelled: %lf \n", distance_travelled);
		printf("\n distancia_path_factor: %lf \n", distance_path_factor);
		printf("Entre: x_prev: %lf y_prev: %lf x_atual: %lf  y_atual: %lf  \n", detailed_goal_list[index].x, detailed_goal_list[index].y, detailed_goal_list[index2].x, detailed_goal_list[index2].y);
		printf("Ponto: x_atual: %lf  y_atual: %lf  \n", path[i].x, path[i].y);
		printf("distance: %lf \n", distance);
		printf("Total: %lf \n", distance);
#endif

	}

	return (total_distance / path.size());
}


double
my_f(const gsl_vector *x, void *params)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;

//	double *p = my_params->par;

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(x, my_params);
	TrajectoryLookupTable::TrajectoryDimensions td;

	if (tcp.tf < 0.2) // o tempo nao pode ser pequeno demais
		tcp.tf = 0.2;

	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->phi_i, false);

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

//Compute cost function to optmize lane
double
my_g(const gsl_vector *x, void *params)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;

		TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(x, my_params);
		TrajectoryLookupTable::TrajectoryDimensions td;

		if (tcp.tf < 0.2) // o tempo nao pode ser pequeno demais
			tcp.tf = 0.2;

		vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->phi_i, false);

		my_params->tcp_seed->vf = tcp.vf;
		my_params->tcp_seed->sf = tcp.sf;

		//TODO Passar tcp.sf (quando estiver correto)
		double total_interest_dist = compute_interest_dist(my_params->detailed_goal_list, path, my_params->lane_sf);
		double dist_objectve = total_interest_dist - 0.2;
		if(dist_objectve < 0.0 )
			dist_objectve = 0.0;

		//double lane_w = (first_plan) ? 0.0001 : 0.00001;

		double result = sqrt((dist_objectve));

	//#ifdef DEBUG_LANE
	//      printf("TD.Dist: %lf \t TD.YAW: %lf \t TD.THETA: %lf \n",(td.dist - my_params->target_td->dist), (carmen_normalize_theta(td.d_yaw) - my_params->target_td->d_yaw), (carmen_normalize_theta(td.theta) - my_params->target_td->theta));
	//      printf("Distance to Lane: %lf \n", total_interest_dist);
	//      printf("Dist_object: %lf \n", dist_objectve);
	//      printf("Result: %lf \n", result);
	//      getchar();
	//#endif
	return result;

}

/* Compute both g and df together. for while df equal to dg */
void
my_gdf(const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_g(x, params);
	my_df(x, params, df);
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
add_points_to_goal_list_interval(carmen_ackerman_path_point_t p1, carmen_ackerman_path_point_t p2, vector<carmen_ackerman_path_point_t> &detailed_goal_list, double *lane_sf, bool end_of_list)
{

		//double theta;
		double delta_x, delta_y, distance;
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

		for(i = 0; i < num_points; i++)
		{
			carmen_ackerman_path_point_t new_point = {p1.x, p1.y, p1.theta, p1.v, p1.phi, 0.0};

			new_point.x = p1.x + i * delta_x;
			new_point.y = p1.y + i * delta_y;
			if(new_point.x >= 0.0)
			{
				if(!detailed_goal_list.empty())
					*lane_sf += sqrt((((delta_x) * (delta_x))) + ((delta_y) * (delta_y)));
				detailed_goal_list.push_back(new_point);
			}
		}
		if(end_of_list){
			*lane_sf += dist(detailed_goal_list.back(), p2);
			detailed_goal_list.push_back(p2);
		}
}


void
build_detailed_goal_list(vector<carmen_ackerman_path_point_t> *lane_in_local_pose, vector<carmen_ackerman_path_point_t> &detailed_goal_list, double *lane_sf)
{
	bool end_of_list = false;
	//printf("lane size dentro do build: %d \n", message->number_of_poses);
	if (lane_in_local_pose->size() > 0)
	{
		for (int i = 0; i < (lane_in_local_pose->size() - 1); i++)
		{
			if(i == (lane_in_local_pose->size() - 2))
				end_of_list = true;
			//printf("p1: %lf %lf p2: %lf %lf\n", message->poses[i].x, message->poses[i].y, message->poses[i + 1].x, message->poses[i + 1].y);
			add_points_to_goal_list_interval(lane_in_local_pose->at(i), lane_in_local_pose->at(i+1), detailed_goal_list, lane_sf, end_of_list);
			//getchar();
		}

	//printf("lane size dentro do build: %lu \n", detailed_goal_list.size());
	}
	else
		printf(KGRN "+++++++++++++ ERRO MENSAGEM DA LANE POSES !!!!\n" RESET);
}


void
optimized_lane_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, vector<carmen_ackerman_path_point_t> *lane_in_local_pose)
{

	// A f(x) muntidimensional que queremos minimizar é:
	//   f(x) = ||(car_simulator(x) - lane(x) < max_dist_lane)||
	// e as dimensões de f(x) são (dist, theta, d_yaw, phi_i, v_i, v_f)
	// O resultado ideal é zero, isto é, a saida de car_simulator deve ser igual a distancia max da lane.
	// As variáveis que serão modificadas pela minimização são:
	//   k1, k2 e tf
	// E, durante a minimização:
	//   v_0, v_i e phi_i são constantes
	//   v_0 = v_i
	//   vt, a0, af, t0, tt e sf sao dependentes das demais segundo o TrajectoryVelocityProfile

	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;

	double suitable_acceleration = compute_suitable_acceleration(tcp_seed, target_td, target_v);

	double lane_sf = 0.0;
	ObjectiveFunctionParams params;
	build_detailed_goal_list(lane_in_local_pose, params.detailed_goal_list, &lane_sf);

	//printf("lane size depois Build: %lu \n", params.detailed_goal_list.size());
	//printf("detailed x: %lf y: %lf \n",detailed_goal_list[detailed_goal_list.size()-1].x, detailed_goal_list[detailed_goal_list.size()-1].y);
	//	getchar();


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
	params.lane_sf = lane_sf;


	gsl_vector *x;
	gsl_multimin_function_fdf my_func;

	my_func.n = 3;
	my_func.f = my_g;
	my_func.df = my_df;
	my_func.fdf = my_gdf;
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
	double MAX_LANE_DIST = 0.3;
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

	} while ((s->f > MAX_LANE_DIST) && (status == GSL_CONTINUE) && (iter < 300)); //alterado de 0.005

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);

	//TODO Verificar esse teste para a lane
	if ((tcp.tf < 0.2) || (s->f > 0.05)) // too short plan or bad minimum (s->f should be close to zero) mudei de 0.05 para outro
		tcp.valid = false;

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

}


// TODO optimizer
TrajectoryLookupTable::TrajectoryControlParameters
get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v)
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

	ObjectiveFunctionParams params;

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
//		print_path(simulate_car_from_parameters(target_td, tcp, target_td.phi_i));
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
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, vector<carmen_ackerman_path_point_t> *lane_in_local_pose)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp_complete;
	tcp_complete = get_optimized_trajectory_control_parameters(tcp_seed, target_td, target_v);
	//optimized_lane_trajectory_control_parameters(tcp_complete, target_td, target_v, lane_in_local_pose);

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
    TrajectoryLookupTable::TrajectoryDimensions target_td = convert_to_trajectory_dimensions(tdd, tcp_seed);
    TrajectoryLookupTable::TrajectoryControlParameters tcp = get_optimized_trajectory_control_parameters(tcp_seed, target_td, target_v);

    return (tcp);
}



TrajectoryLookupTable::TrajectoryControlParameters
get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
        TrajectoryLookupTable::TrajectoryDiscreteDimensions &tdd, double target_v, vector<carmen_ackerman_path_point_t> optimized_path)
{
    TrajectoryLookupTable::TrajectoryDimensions target_td = convert_to_trajectory_dimensions(tdd, tcp_seed);
    TrajectoryLookupTable::TrajectoryControlParameters tcp = get_optimized_trajectory_control_parameters(tcp_seed, target_td, target_v);
    if (tcp.valid)
    {
		TrajectoryLookupTable::TrajectoryDimensions td;
		optimized_path = simulate_car_from_parameters(td, tcp, tdd.phi_i, false);
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
get_trajectory_dimensions_from_robot_state(Pose *localize_pose, Command last_odometry,	Pose *goal_pose)
{
	TrajectoryLookupTable::TrajectoryDimensions td;

	td.dist = sqrt((goal_pose->x - localize_pose->x) * (goal_pose->x - localize_pose->x) +
					(goal_pose->y - localize_pose->y) * (goal_pose->y - localize_pose->y));
	td.theta = carmen_normalize_theta(atan2(goal_pose->y - localize_pose->y, goal_pose->x - localize_pose->x) - localize_pose->theta);
	td.d_yaw = carmen_normalize_theta(goal_pose->theta - localize_pose->theta);
	td.phi_i = last_odometry.phi;
	td.v_i = last_odometry.v;

	return (td);
}


//TODO verificar conversão
bool
move_lane_robot_reference_system(Pose *localize_pose, carmen_rddf_road_profile_message *goal_list_message,
								Pose *goal_pose, vector<carmen_ackerman_path_point_t> *lane_in_local_pose)
{
	double last_dist = DBL_MAX;
	double dist = 0.0;

	SE2 robot_pose(localize_pose->x, localize_pose->y, localize_pose->theta);
	SE2 goal_in_world_reference(goal_pose->x, goal_pose->y, goal_pose->theta);
	SE2 goal_in_car_reference = robot_pose.inverse() * goal_in_world_reference;
	double goal_x = goal_in_car_reference[0];
	double goal_y = goal_in_car_reference[1];

	if (goal_list_message->number_of_poses < 2)
		return false;

	int indice = 0;
	if (goal_list_message->poses[0].x == goal_list_message->poses[1].x && goal_list_message->poses[0].y == goal_list_message->poses[1].y)
		indice = 1;

	for (int i = 0; indice < goal_list_message->number_of_poses; i++ , indice++)
	{

		SE2 lane_in_world_reference(goal_list_message->poses[indice].x, goal_list_message->poses[indice].y, goal_list_message->poses[indice].theta);
		SE2 lane_in_car_reference = robot_pose.inverse() * lane_in_world_reference;


		carmen_ackerman_path_point_t local_reference_lane_point = {lane_in_car_reference[0], lane_in_car_reference[1], lane_in_car_reference[2],
				goal_list_message->poses[indice].v, goal_list_message->poses[indice].phi, 0.0};

		lane_in_local_pose->push_back(local_reference_lane_point);

//		if(i < 4)
//			printf("%d: Global Lane: x: %lf y: %lf \t Goal x: %lf y:%lf \n", i,lane_in_local_pose->at(i).x, lane_in_local_pose->at(i).y, goal_pose->x, goal_pose->y);
		if (local_reference_lane_point.x == goal_x && local_reference_lane_point.y == goal_y)
		{
			return true;
		}
		dist = sqrt((carmen_square(lane_in_local_pose->at(i).x - goal_x) + carmen_square(lane_in_local_pose->at(i).y - goal_y)));
		if (last_dist < dist)
			return false;
		last_dist = dist;
	}
	return false;
}


int
find_pose_in_path(double &time_displacement, Pose *localize_pose, vector<carmen_ackerman_path_point_t> path)
{
	time_displacement = 0.0;
	carmen_ackerman_path_point_t p;
	unsigned int i;

	for (i = 0; i < (path.size() - 1); i++)
		if (Util::between(p, localize_pose, path[i], path[i+1]))
			break;

	time_displacement = p.time;

	return (i);
}


double
compute_delay(vector<carmen_ackerman_path_point_t> path, int path_index,
		double localize_pose_timestamp, double path_timestamp, double time_displacement)
{
	double path_time_up_to_localize_pose = path_timestamp;
	for (int i = 0; i < path_index; i++)
		path_time_up_to_localize_pose += path[i].time;
	path_time_up_to_localize_pose += time_displacement;

	double delay = localize_pose_timestamp - path_time_up_to_localize_pose;

	return (delay);
}


Pose
predicted_current_pose(Pose *localize_pose, double localize_pose_timestamp, Command last_odometry,
		vector<carmen_ackerman_path_point_t> path, double path_timestamp)
{
	double time_displacement;
	int path_index = find_pose_in_path(time_displacement, localize_pose, path);
	double delay = compute_delay(path, path_index, localize_pose_timestamp, path_timestamp, time_displacement);

	unsigned int i = path_index;
	double delta_t = path[i].time - time_displacement;
	double total_t = 0.0;
	double distance_traveled = 0.0;

	Robot_State robot_state;
	robot_state.pose = *localize_pose;
	robot_state.v_and_phi = last_odometry;
	Command requested_command;
	do
	{
		requested_command.v = path[i].v;
		requested_command.phi = path[i].phi;
		TrajectoryLookupTable::predict_next_pose(robot_state, requested_command,
				delta_t, &distance_traveled, delta_t);
		total_t += delta_t;
		i++;
		if (path.size() < (i + 1))
		{
			std::cout << "Error in predicted_current_pose()! path.size() = " << path.size()
					<< "  i = " << i
					<< "  path_index = " << path_index
					<< "  delay = " << delay
					<< "  total_t = " << total_t
					<< std::endl;
			exit(1);
		}
		delta_t = path[i].time;
	} while ((total_t + delta_t) < delay);

	requested_command.v = path[i].v;
	requested_command.phi = path[i].phi;
	TrajectoryLookupTable::predict_next_pose(robot_state, requested_command,
			delay - total_t, &distance_traveled, delay - total_t);
	std::cout << "#### delay = " << delay << std::endl;

	return (robot_state.pose);
}


Pose
predict_plan_initial_pose(vector<TrajectoryLookupTable::Plan> &previous_plans,
		Pose *localize_pose, double localize_pose_timestamp, Command last_odometry)
{
	vector<TrajectoryLookupTable::Plan>::iterator it = previous_plans.begin();
	std::cout << "previous_plans.size() = " << previous_plans.size() << std::endl;
	while (it != previous_plans.end())
	{
        if (it->path.size() > 2)
        {
            if (Util::between(localize_pose, it->path[1], it->path[2])) // good!
            {
                std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@ break!!!" << std::endl;
                break;
            }
        }
//		if (it->path.size() > 2)
//            for (int i = 0; i < (it->path.size() - 1); i++)
//                if (Util::between(localize_pose, it->path[i], it->path[i + 1]))
//                    std::cout << "$$$$$$$$$$$$$$ between = " << i << std::endl;
		//std::cout << "it->path.size() = " << it->path.size() << std::endl;
		if ((localize_pose_timestamp - it->timestamp) > 2.0) // too old
		{
			it = previous_plans.erase(it);
			std::cout << "erase 1 " << std::endl;

			continue;
		}
		if (it->path.size() > 3)
		{
			if (Util::distance(localize_pose, &(it->path[2])) > Util::distance(localize_pose, &(it->path[3]))) // too old
			{
				it = previous_plans.erase(it);
				std::cout << "erase 2 " << std::endl;
				continue;
			}
		}
		it++;
	}
	std::cout << "num plans = " << previous_plans.size() << std::endl;
	if (it == previous_plans.end())
		return (*localize_pose);
	else
		return (predicted_current_pose(localize_pose, localize_pose_timestamp, last_odometry, it->path, it->timestamp));
}


//double
//compute_system_delay(vector<TrajectoryLookupTable::Plan> &previous_plans,
//        Pose *localize_pose, double localize_pose_timestamp, Command last_odometry, double plan_delay)
//{
//    vector<TrajectoryLookupTable::Plan>::iterator it, it_found;
//    it = previous_plans.begin();
//    std::cout << "previous_plans.size() = " << previous_plans.size() << std::endl;
//    int i;
//    while (it != previous_plans.end())
//    {
//        if ((localize_pose_timestamp - it->timestamp) > 2.0) // too old
//        {
//            it = previous_plans.erase(it);
//            continue;
//        }
//        if (it->path.size() > 2)
//        {
//            bool found = false;
//            for (i = 0; i < (it->path.size() - 1); i++)
//            {
//                if (Util::between(localize_pose, it->path[i], it->path[i + 1]))
//                {
//                    found = true;
//                    it_found = it;
//                    break;
//                }
//            }
//            if (found)
//                break;
//        }
//        it++;
//    }
//    if (it == previous_plans.end())
//    {
//        std::cout << "delay short" << std::endl;
//        return (plan_delay);
//    }
//    else
//    {
//        double time_displacement;
//        int path_index = find_pose_in_path(time_displacement, localize_pose, it_found->path);
//        double delay = compute_delay(it_found->path, i, localize_pose_timestamp, it_found->timestamp, 0);
//        std::cout << "long delay" << std::endl;
//        return (plan_delay + delay);
//    }
//}
//
//
//vector<carmen_ackerman_path_point_t>
//TrajectoryLookupTable::compute_path_to_goal(Pose *localize_pose, Pose *goal_pose, Command last_odometry,
//		double target_v, double localize_pose_timestamp)
//{
//	static vector<TrajectoryLookupTable::Plan> previous_plans;
//	double planner_initial_time = carmen_get_time();
//
//	//Pose predicted_pose = predict_plan_initial_pose(previous_plans, localize_pose, localize_pose_timestamp, last_odometry);
//	TrajectoryLookupTable::TrajectoryDimensions td = get_trajectory_dimensions_from_robot_state(localize_pose, last_odometry, goal_pose);
//	TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd = get_discrete_dimensions(td);
//	vector<carmen_ackerman_path_point_t> path;
//	if (has_valid_discretization(tdd))
//	{
//		TrajectoryLookupTable::TrajectoryControlParameters tcp = search_lookup_table(tdd);
//		if (tcp.valid)
//		{
//			tcp = get_optimized_trajectory_control_parameters(tcp, td, target_v);
//			path = simulate_car_from_parameters(td, tcp, td.phi_i);
//		}
//		else
//		{
//			// printf("@@@@@@@@@@@ Could not find a valid entry in the table!!!!\n");
//			return (path); // empty path
//		}
//
//		move_path_to_current_robot_pose(path, localize_pose);
//		double system_delay = compute_system_delay(previous_plans, localize_pose,
//		        localize_pose_timestamp, last_odometry, carmen_get_time() - planner_initial_time);
//		std::cout << "$$$$$$$$$$$$$$ system_delay = " << system_delay << std::endl;
//		TrajectoryLookupTable::Plan plan;
//		plan.path = path;
//		plan.timestamp = localize_pose_timestamp;
//		previous_plans.push_back(plan);
//		double delay = 0.0;
//		int index;
//        for (index = 0; (delay < 0.7) && (index < path.size()); index++)
//            delay += path[index].time;
//        std::cout << "$$$$$$$$$$$$$$ index = " << index << std::endl;
//        for (int j = 0; j < index; j++)
//            path.erase(path.begin());
////        for (int i = 0; (i < 12) && (path.size() > 1); i++)
////            path.erase(path.begin());
//
//		return (path);
//	}
//	else
//	{
//		printf("Invalid discretization!!!!\n");
//		return (path);
//	}
//}


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
remove_path_with_collision(vector<vector<carmen_ackerman_path_point_t> >& path)
{
	carmen_point_t pose;
	carmen_robot_ackerman_config_t car_config;

	car_config.distance_between_rear_car_and_rear_wheels =
			GlobalState::robot_config.distance_between_rear_car_and_rear_wheels;
	car_config.length = GlobalState::robot_config.length * 1.1;
	car_config.width = GlobalState::robot_config.width * 1.1;

	for (unsigned int j = 0; j < path.back().size(); j++)
	{
		pose.x = path.back()[j].x;
		pose.y = path.back()[j].y;
		pose.theta = path.back()[j].theta;
		if (pose_hit_obstacle(pose, &GlobalState::cost_map, &car_config))
		{
			//printf("---------- HIT OBSTACLE!!!!\n");
			path.pop_back();
			return (true);
		}
	}
	return (false);
}


void
put_shorter_path_in_front(vector<vector<carmen_ackerman_path_point_t> > &path, int shorter_path)
{
	if (path.size() > 1)
	{
		vector<carmen_ackerman_path_point_t> shoter_path;
		shoter_path = path[shorter_path];
		path.erase(path.begin() + shorter_path);
		path.insert(path.begin(), shoter_path);
	}
}


//todo
void
compute_paths(const vector<Command> &lastOdometryVector, vector<Pose> &goalPoseVector, double target_v,
		Pose *localize_pose, vector<vector<carmen_ackerman_path_point_t> > &path,
		carmen_rddf_road_profile_message *goal_list_message)
{
//----verifica se chegou uma lane nova
//	printf("lane size assim que chegou: %d \n", goal_list_message->number_of_poses);
	vector<carmen_ackerman_path_point_t> lane_in_local_pose;
	bool goal_in_lane = false;
	goal_in_lane = move_lane_robot_reference_system(localize_pose, goal_list_message, &goalPoseVector[0], &lane_in_local_pose);
	if(!goal_in_lane)
	{
		lane_in_local_pose.clear();
	}

	FILE *problems;
	problems = fopen("problems.txt", "a");

//	int path_order = 0;
//	int shorter_path = -1;
//	double shorter_path_size = 1000.0;

	int errors = 0;
	for (unsigned int i = 0; i < lastOdometryVector.size(); i++)
	{
		for (unsigned int j = 0; j < goalPoseVector.size(); j++)
		{
			TrajectoryLookupTable::TrajectoryDimensions td = get_trajectory_dimensions_from_robot_state(localize_pose, lastOdometryVector[i], &goalPoseVector[j]);
			TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd = get_discrete_dimensions(td);
			if (!has_valid_discretization(tdd))
			{
				printf("Invalid discretization!!!!\n");
				write_tdd_to_file(problems, tdd, "Invalid discretization: ");
				continue;
			}
			TrajectoryLookupTable::TrajectoryControlParameters tcp = search_lookup_table(tdd);
			if (!tcp.valid)
			{
				printf(KMAG "@@@@@@@@@@@ Could not find a valid entry in the table!!!!\n\033[0m");
				write_tdd_to_file(problems, tdd, "Could not find: ");
				continue;
			}
//TODO concluir otimizadores
			TrajectoryLookupTable::TrajectoryControlParameters otcp;
			otcp = get_complete_optimized_trajectory_control_parameters(tcp, td, target_v, &lane_in_local_pose);
			//otcp = get_optimized_trajectory_control_parameters(tcp, td, target_v, &lane_in_local_pose);
			//Lane optmized

			if (otcp.valid)
			{
				path.push_back(simulate_car_from_parameters(td, otcp, td.phi_i));
				if (path_has_loop(td.dist, otcp.sf))
				{
					path.pop_back();
					printf(KRED "+++++++++++++ Path had loop...\n" RESET);
					write_tdd_to_file(problems, tdd, "Path had loop: ");
					continue;
				}

				move_path_to_current_robot_pose(path.back(), localize_pose);
				apply_system_delay(path.back());

				remove_path_with_collision(path);
//				if (!remove_path_with_collision(path) && (otcp.sf < shorter_path_size))
//				{
//					shorter_path_size = otcp.sf;
//					shorter_path = path_order;
//					path_order++;
//				}
			}
			else
			{
//				// dist: 3, theta: 10, phi_i: 7, v_i: 3, d_yaw: 10
				if (tdd.dist == 3 && tdd.theta == 10 && tdd.phi_i == 7 && tdd.v_i == 3 && tdd.d_yaw == 10)
					errors++;
				write_tdd_to_file(problems, tdd, "Could NOT optimize: ");
				printf(KYEL "+++++++++++++ Could NOT optimize %d !!!!\n" RESET, errors);
			}
		}

		if (path.size() > 0) // If could not find a good path, try swerve
			break;
	}
	// put_shorter_path_in_front(path, shorter_path);

	fclose(problems);
}


vector<vector<carmen_ackerman_path_point_t>>
TrajectoryLookupTable::compute_path_to_goal(Pose *localize_pose, Pose *goal_pose, Command last_odometry,
        double target_v, carmen_rddf_road_profile_message *goal_list_message)
{

	vector<vector<carmen_ackerman_path_point_t>> path;
    vector<Command> lastOdometryVector;
    vector<Pose> goalPoseVector;
    vector<int> magicSignals = {0, 1, -1, 2, -2, 3, -3,  4, -4,  5, -5};

    // @@@ Tranformar os dois loops abaixo em uma funcao -> compute_alternative_path_options()
    for (int i = 0; i < 1; i++)
    {
    	Command newOdometry = last_odometry;
    	newOdometry.phi +=  0.15 * magicSignals[i]; //(0.5 / (newOdometry.v + 1))
    	lastOdometryVector.push_back(newOdometry);
    }

    for (int i = 0; i < 1; i++)
    {
    	//printf("Goal x: %lf Goal y: %lf \n",goal_pose->x, goal_pose->y);
    	Pose newPose = *goal_pose;
    	newPose.x += magicSignals[i] / 5.0 * cos((goal_pose->theta) - carmen_degrees_to_radians(90.0));
    	newPose.y += magicSignals[i] / 5.0 * sin((goal_pose->theta) - carmen_degrees_to_radians(90.0));
    	goalPoseVector.push_back(newPose);
    }

	compute_paths(lastOdometryVector, goalPoseVector, target_v, localize_pose, path, goal_list_message);

	return (path);
}

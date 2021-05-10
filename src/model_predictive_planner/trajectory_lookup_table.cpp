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
#include <string>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <car_model.h>
#include "trajectory_lookup_table.h"
#include "model_predictive_planner_optimizer.h"

extern int use_unity_simulator;
//TODO
//#define DEBUG_LANE


void
plot_state(vector<carmen_ackerman_path_point_t> &pOTCP, vector<carmen_ackerman_path_point_t> &pLane,
		  vector<carmen_ackerman_path_point_t> &pSeed, std::string titles[])
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

	for (unsigned int i = 0; i < pOTCP.size(); i++)
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %lf %lf %lf\n", pOTCP.at(i).x, pOTCP.at(i).y, 1.0 * cos(pOTCP.at(i).theta), 1.0 * sin(pOTCP.at(i).theta), pOTCP.at(i).theta, pOTCP.at(i).phi, pOTCP.at(i).time);
	if(!pLane.empty())
		for (unsigned int i = 0; i < pLane.size(); i++)
			fprintf(gnuplot_data_lane, "%lf %lf %lf %lf %lf %lf %lf\n", pLane.at(i).x, pLane.at(i).y, 1.0 * cos(pLane.at(i).theta), 1.0 * sin(pLane.at(i).theta), pLane.at(i).theta, pLane.at(i).phi, pLane.at(i).time);

	for (unsigned int i = 0; i < pSeed.size(); i++)
		fprintf(gnuplot_data_seed, "%lf %lf\n", pSeed.at(i).x, pSeed.at(i).y);

	fclose(gnuplot_data_file);
	fclose(gnuplot_data_lane);
	fclose(gnuplot_data_seed);

//	fprintf(gnuplot_pipe, "unset arrow\nset arrow from %lf, %lf to %lf, %lf nohead\n",0, -60.0, 0, 60.0);

	fprintf(gnuplot_pipeMP, "plot "
			"'./gnuplot_data.txt' using 1:2:3:4 w vec size  0.3, 10 filled title '%s',"
			"'./gnuplot_data_lane.txt' using 1:2:3:4 w vec size  0.3, 10 filled title '%s',"
			"'./gnuplot_data_seed.txt' using 1:2 with lines title '%s' axes x1y1\n", titles[0].c_str(), titles[1].c_str(), titles[2].c_str());

	fflush(gnuplot_pipeMP);
}


TrajectoryLookupTable::TrajectoryControlParameters_old trajectory_lookup_table_old[N_DIST][N_THETA][N_D_YAW][N_I_PHI][N_I_V];
TrajectoryLookupTable::TrajectoryControlParameters trajectory_lookup_table[N_DIST][N_THETA][N_D_YAW][N_I_PHI][N_I_V];


TrajectoryLookupTable::TrajectoryLookupTable(int update_lookup_table)
{
	// Codigo para gerar nova tabela a partir de uma tabela velha quando de mudancas na estrutura da seed. Quando usar, rodar o model_predictive_planner no seu diretorio.
//	load_trajectory_lookup_table_old();
//	save_trajectory_lookup_table();

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
search_lookup_table_old(TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd)
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
search_lookup_table(TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd)
{
	// TODO: pegar a media de todas as leituras ponderada pela distancia para o td.
	// Tem que passar o td ao inves do tdd.
	TrajectoryLookupTable::TrajectoryControlParameters tcp;
	tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];

	if (tcp.valid)
		return (tcp);

	int dist = tdd.dist;
	for (int i = -4; i < 5; i++)
	{
		tdd.dist = dist + i;
		if (has_valid_discretization(tdd))
		{
			tcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
			if (tcp.valid)
				return (tcp);
		}
	}
	tdd.dist = dist;

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

	tcp = trajectory_lookup_table[tdd.dist][ZERO_THETA_I][ZERO_D_YAW_I][tdd.phi_i][tdd.v_i];
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
	tcp.tt = 0.0;
	tcp.k1 = 0.0;
	tcp.k2 = 0.0;
	tcp.k3 = 0.0;
	tcp.has_k1 = false;
	tcp.shift_knots = false;
	tcp.a = 0.0;
	tcp.vf = 0.0;
	tcp.sf = 0.0;
	tcp.s = 0.0;

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
TrajectoryLookupTable::load_trajectory_lookup_table_old()
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
						{
							fread((void *) &(trajectory_lookup_table_old[i][j][k][l][m]),
									sizeof(TrajectoryLookupTable::TrajectoryControlParameters_old), 1, tlt_f);
							trajectory_lookup_table[i][j][k][l][m].valid = trajectory_lookup_table_old[i][j][k][l][m].valid;
							trajectory_lookup_table[i][j][k][l][m].tt = trajectory_lookup_table_old[i][j][k][l][m].tt;
							trajectory_lookup_table[i][j][k][l][m].k2 = trajectory_lookup_table_old[i][j][k][l][m].k2;
							trajectory_lookup_table[i][j][k][l][m].k3 = trajectory_lookup_table_old[i][j][k][l][m].k3;
							trajectory_lookup_table[i][j][k][l][m].k1 = trajectory_lookup_table_old[i][j][k][l][m].k1;
							trajectory_lookup_table[i][j][k][l][m].has_k1 = trajectory_lookup_table_old[i][j][k][l][m].has_k1;
							trajectory_lookup_table[i][j][k][l][m].a = trajectory_lookup_table_old[i][j][k][l][m].a;
							trajectory_lookup_table[i][j][k][l][m].vf = trajectory_lookup_table_old[i][j][k][l][m].vf;
							trajectory_lookup_table[i][j][k][l][m].sf = trajectory_lookup_table_old[i][j][k][l][m].sf;
							trajectory_lookup_table[i][j][k][l][m].s = trajectory_lookup_table_old[i][j][k][l][m].s;

							trajectory_lookup_table[i][j][k][l][m].shift_knots = false;
						}
		fclose(tlt_f);

		return (true);
	}
	else
		return (false);
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
get_k2_by_index(int index)
{
	return (geometric_progression(index, FIRST_K2, RATIO_K2, ZERO_K2_I));
}


float
get_k3_by_index(int index)
{
	return (geometric_progression(index, FIRST_K3, RATIO_K3, ZERO_K3_I));
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
generate_trajectory_control_parameters_sample(int k2, int k3, int i_v, int dist)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp;

	tcp.valid = true;

	tcp.k1 = 0.0;
	tcp.has_k1 = false;
	tcp.k2 = get_k2_by_index(k2);
	tcp.k3 = get_k3_by_index(k3);

	double distance = get_distance_by_index(dist);
	double v0 = get_initial_velocity_by_index(i_v);
	// s = s0 + v0.t + 1/2.a.t^2
			// s0 = 0; v0 = tcp.v0; t = PROFILE_TIME
	tcp.a = (2.0 * (distance - v0 * PROFILE_TIME)) / (PROFILE_TIME * PROFILE_TIME);
	tcp.vf = v0 + tcp.a * PROFILE_TIME;
	tcp.tt = PROFILE_TIME;

	return (tcp);
}


TrajectoryLookupTable::TrajectoryControlParameters
generate_trajectory_control_parameters_sample(double k2, double k3, int i_v, int dist)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp;

	tcp.valid = true;

	tcp.k2 = k2;
	tcp.k3 = k3;

	double v0 = get_initial_velocity_by_index(i_v);

	double distance = get_distance_by_index(dist);

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
	tcp.a = (distance - v0 * time) / (time * time * 0.5);
	if (v0 < 0) //reverse mode
		tcp.a = (-1)*tcp.a;

	// v = v0 + a.t
	tcp.vf = v0 + tcp.a * time;
	tcp.tt = time;
	tcp.s = distance;

	return (tcp);
}


TrajectoryLookupTable::TrajectoryControlParameters
generate_random_trajectory_control_parameters_sample(int i_v, int dist)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp;

	double v0 = get_initial_velocity_by_index(i_v);
	float distance = get_distance_by_index(dist);

	// distance = delta_t * (v0 + vt) / 2 -> trapezoidal rule
	tcp.vf = 2.0 * distance / PROFILE_TIME - v0;
	tcp.tt = PROFILE_TIME;
	tcp.a = (tcp.vf - v0) / tcp.tt;

	tcp.k2 = get_k2_by_index(get_random_integer(N_K2));
	tcp.k3 = get_k3_by_index(get_random_integer(N_K3));

	tcp.valid = true;

	return (tcp);
}


carmen_ackerman_path_point_t
convert_to_carmen_ackerman_path_point_t(const carmen_ackerman_traj_point_t robot_state, const double time)
{
	carmen_ackerman_path_point_t path_point;

	path_point.x = robot_state.x;
	path_point.y = robot_state.y;
	path_point.theta = robot_state.theta;
	path_point.v = robot_state.v;
	path_point.phi = robot_state.phi;
	path_point.time = time;

	return (path_point);
}


carmen_robot_and_trailer_path_point_t
convert_to_carmen_robot_and_trailer_path_point_t(const carmen_robot_and_trailer_traj_point_t robot_state, const double time)
{
	carmen_robot_and_trailer_path_point_t path_point;

	path_point.x = robot_state.x;
	path_point.y = robot_state.y;
	path_point.theta = robot_state.theta;
	path_point.beta = robot_state.beta;
	path_point.v = robot_state.v;
	path_point.phi = robot_state.phi;
	path_point.time = time;

	return (path_point);
}


vector<carmen_ackerman_path_point_t>
apply_robot_delays(vector<carmen_ackerman_path_point_t> &original_path)
{
	// Velocity delay
	vector<carmen_ackerman_path_point_t> path = original_path;
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


//static void
//print_path_(vector<carmen_ackerman_path_point_t> path)
//{
//	for (unsigned int i = 0; (i < path.size()) && (i < 15); i++)
//		printf("v %5.3lf, phi %5.3lf, t %5.3lf, x %5.3lf, y %5.3lf, theta %5.3lf\n",
//				path[i].v, path[i].phi, path[i].time,
//				path[i].x - GlobalState::localizer_pose->x, path[i].y - GlobalState::localizer_pose->y,
//				path[i].theta);
//
//	printf("\n");
//	fflush(stdout);
//}


double
compute_path_via_simulation(carmen_robot_and_trailer_traj_point_t &robot_state, Command &command,
		vector<carmen_robot_and_trailer_path_point_t> &path,
		TrajectoryLookupTable::TrajectoryControlParameters tcp,
		gsl_spline *phi_spline, gsl_interp_accel *acc, double v0, double i_phi, double delta_t)
{
	int i = 0;
	double t, last_t;
	double distance_traveled = 0.0;
	//double delta_t = 0.075;
	int reduction_factor;
	if (use_unity_simulator || GlobalState::eliminate_path_follower)
		reduction_factor = 1;
	else
		reduction_factor = 1 + (int)((tcp.tt / delta_t) / 90.0);

	robot_state.x = 0.0;
	robot_state.y = 0.0;
	robot_state.theta = 0.0;
	robot_state.v = v0;
	robot_state.phi = i_phi;

	command.v = v0;
	carmen_robot_and_trailer_traj_point_t last_robot_state = robot_state;
	for (last_t = t = 0.0; t < (tcp.tt - delta_t); t += delta_t)
	{
		command.v += tcp.a * delta_t;
		command.phi = gsl_spline_eval(phi_spline, t + delta_t, acc);

		robot_state = carmen_libcarmodel_recalc_pos_ackerman(robot_state, command.v, command.phi, delta_t,
				&distance_traveled, delta_t, GlobalState::robot_config, GlobalState::semi_trailer_config);
		if ((i % reduction_factor) == 0)
		{	// Cada ponto na trajetoria marca uma posicao do robo e o delta_t para chegar aa proxima
			path.push_back(convert_to_carmen_robot_and_trailer_path_point_t(last_robot_state, t + delta_t - last_t));
			last_robot_state = robot_state;
			last_t = t + delta_t;
		}
		i++;
	}

	if (((tcp.tt - t) > 0.0)) // && (command.v > 0.0))
	{
		double final_delta_t = tcp.tt - t;
		command.phi = gsl_spline_eval(phi_spline, tcp.tt, acc);
		command.v += tcp.a * final_delta_t;

		robot_state = carmen_libcarmodel_recalc_pos_ackerman(robot_state, command.v, command.phi,
				final_delta_t, &distance_traveled, final_delta_t, GlobalState::robot_config, GlobalState::semi_trailer_config);
		// Cada ponto na trajetoria marca uma posicao do robo e o delta_t para chegar aa proxima
		path.push_back(convert_to_carmen_robot_and_trailer_path_point_t(last_robot_state, tcp.tt - last_t));
		// A ultima posicao nao tem proxima, logo, delta_t = 0.0
		path.push_back(convert_to_carmen_robot_and_trailer_path_point_t(robot_state, 0.0));
	}

//	if (GlobalState::eliminate_path_follower)
//	{
//		path = apply_robot_delays(path);
//
////		robot_state.v = v0;
////		robot_state.phi = i_phi;
////		path[0] = convert_to_carmen_ackerman_path_point_t(robot_state, delta_t);
//
//		robot_state = {0.0, 0.0, 0.0, path[0].v, path[0].phi};
////		robot_state = {path[0].x, path[0].y, path[0].theta, path[0].v, path[0].phi};
//		distance_traveled = 0.0;
//		for (unsigned int i = 1; i < path.size(); i++)
//		{
//			robot_state = carmen_libcarmodel_recalc_pos_ackerman(robot_state, path[i].v, path[i].phi,
//					path[i].time, &distance_traveled, path[i].time, GlobalState::robot_config);
//			path[i] = convert_to_carmen_ackerman_path_point_t(robot_state, delta_t);
//		}
//	//	printf("Depois\n\n");
//	//	print_path_(path);
//	}

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


double
get_max_distance_in_path(vector<carmen_robot_and_trailer_path_point_t> path, carmen_robot_and_trailer_path_point_t &furthest_point)
{
	double max_dist = 0.0;

	furthest_point = path[0];
	for (unsigned int i = 0; i < path.size(); i++)
	{
		double distance = DIST2D(path[0], path[i]);
		if (distance > max_dist)
		{
			max_dist = distance;
			furthest_point = path[i];
		}
	}

	return (max_dist);
}


vector<carmen_robot_and_trailer_traj_point_t>
simulate_car_from_parameters(TrajectoryLookupTable::TrajectoryDimensions &td,
		TrajectoryLookupTable::TrajectoryControlParameters &tcp, double v0, double i_phi,
		bool display_phi_profile, double delta_t)
{
	vector<carmen_robot_and_trailer_path_point_t> path;
	if (!tcp.valid)
	{
//		printf("Warning: invalid TrajectoryControlParameters tcp in simulate_car_from_parameters()\n");
		return (path);
	}

	// Create phi profile

	gsl_interp_accel *acc;
	gsl_spline *phi_spline;
	if (tcp.has_k1)
	{
		if (tcp.shift_knots)
		{
			double knots_x[4] = {0.0, tcp.tt / 2.0, 3.0 * (tcp.tt / 4.0), tcp.tt};
			double knots_y[4] = {i_phi, tcp.k2, tcp.k1, tcp.k3};
			acc = gsl_interp_accel_alloc();
			const gsl_interp_type *type = gsl_interp_cspline;
			phi_spline = gsl_spline_alloc(type, 4);
			gsl_spline_init(phi_spline, knots_x, knots_y, 4);
//			static int i = 0;
//			printf("%d ENTREI!!!\n\n", i++);
		}
		else
		{
			double knots_x[4] = {0.0, tcp.tt / 4.0, tcp.tt / 2.0, tcp.tt};
			double knots_y[4] = {i_phi, tcp.k1, tcp.k2, tcp.k3};
			acc = gsl_interp_accel_alloc();
			const gsl_interp_type *type = gsl_interp_cspline;
			phi_spline = gsl_spline_alloc(type, 4);
			gsl_spline_init(phi_spline, knots_x, knots_y, 4);
		}
	}
	else
	{
		double knots_x[3] = {0.0, tcp.tt / 2.0, tcp.tt};
		double knots_y[3] = {i_phi, tcp.k2, tcp.k3};
		acc = gsl_interp_accel_alloc();
		const gsl_interp_type *type = gsl_interp_cspline;
		phi_spline = gsl_spline_alloc(type, 3);
		gsl_spline_init(phi_spline, knots_x, knots_y, 3);
	}
	print_phi_profile_temp(phi_spline, acc, tcp.tt, display_phi_profile);

	Command command;
	carmen_robot_and_trailer_traj_point_t robot_state;

	double distance_traveled = compute_path_via_simulation(robot_state, command, path, tcp, phi_spline, acc, v0, i_phi, delta_t);

	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);
	carmen_robot_and_trailer_path_point_t furthest_point;
	td.dist = get_max_distance_in_path(path, furthest_point);
	if (GlobalState::reverse_planning)
	{
		td.theta = -carmen_normalize_theta(atan2(furthest_point.y, furthest_point.x) - M_PI);
		td.d_yaw = furthest_point.theta;
	}
	else
	{
		td.theta = atan2(furthest_point.y, furthest_point.x);
		td.d_yaw = furthest_point.theta;
	}
	td.phi_i = i_phi;
	td.v_i = v0;
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
compute_trajectory_dimensions(TrajectoryLookupTable::TrajectoryControlParameters &tcp, int i_v0, int i_phi,
		vector<carmen_ackerman_path_point_t> &path, bool print)
{
	double d_i_phi = get_i_phi_by_index(i_phi);
	double d_i_v0 = get_initial_velocity_by_index(i_v0);
	TrajectoryLookupTable::TrajectoryDimensions td;
	path = simulate_car_from_parameters(td, tcp, d_i_v0, d_i_phi, print);
	if (tcp.valid && print)
		print_path(path);

	return (td);
}


bool
same_tcp(TrajectoryLookupTable::TrajectoryControlParameters tcp1, TrajectoryLookupTable::TrajectoryControlParameters tcp2)
{
	bool result = true;

	if (tcp1.a != tcp2.a)
	{
		//std::cout << "tcp1.a0 = " << tcp1.a0 << "  tcp2.a0 = " << tcp2.a0 << std::endl;
		result = false;
	}
	if (tcp1.k2 != tcp2.k2)
	{
		//std::cout << "tcp1.k2 = " << tcp1.k2 << "  tcp2.k2 = " << tcp2.k2 << std::endl;
		result = false;
	}
	if (tcp1.k3 != tcp2.k3)
	{
		//std::cout << "tcp1.k3 = " << tcp1.k3 << "  tcp2.k3 = " << tcp2.k3 << std::endl;
		result = false;
	}
	if (tcp1.sf != tcp2.sf)
	{
		//std::cout << "tcp1.sf = " << tcp1.sf << "  tcp2.sf = " << tcp2.sf << std::endl;
		result = false;
	}
	if (tcp1.tt != tcp2.tt)
	{
		//std::cout << "tcp1.tt = " << tcp1.tt << "  tcp2.tt = " << tcp2.tt << std::endl;
		result = false;
	}
	if (tcp1.vf != tcp2.vf)
	{
		//std::cout << "tcp1.vf = " << tcp1.vf << "  tcp2.vf = " << tcp2.vf << std::endl;
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


bool
path_has_loop(double dist, double sf)
{
	if (dist < 0.05)
		return (false);

	if (sf > (M_PI * dist * 1.1)) // se sf for maior que meio arco com diametro dist mais um pouco (1.1) tem loop
		return (true);
	return (false);
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
//			for (int k2 = 0; k2 < N_K2; k2++)
//			{
//				for (int k3 = 0; k2 < N_K3; k3++)
//				{
			double delta_k2 = (get_k2_by_index(N_K2-1) - get_k2_by_index(0)) / 60.0;
			for (double k2 = get_k2_by_index(0); k2 < get_k2_by_index(N_K2-1); k2 += delta_k2)
			{
				double delta_k3 = (get_k3_by_index(N_K3-1) - get_k3_by_index(0)) / 60.0;
				for (double k3 = get_k3_by_index(0); k3 < get_k3_by_index(N_K3-1); k3 += delta_k3)
				{
					for (int i_v = 0; i_v < N_I_V; i_v++)
						// Checar se os limites do carro estao
						// sendo obedecidos (ate o phi dentro dos splines?)
						// Otimizar e checar se eh valida a trajetoria usando as facilidades de
						// visualizacao ja implementadas.
					{
						if (i_v < 4)
						{
							GlobalState::reverse_driving_flag = 1;
							GlobalState::reverse_planning = 1;
						}
						else
						{
							GlobalState::reverse_driving_flag = 0;
							GlobalState::reverse_planning = 0;
						}
						//i_v = 7;
						TrajectoryLookupTable::TrajectoryControlParameters tcp = generate_trajectory_control_parameters_sample(k2, k3, i_v, dist);
						vector<carmen_ackerman_path_point_t> path;
						TrajectoryLookupTable::TrajectoryDimensions td = compute_trajectory_dimensions(tcp, i_v, i_phi, path, false);
						//                        td = compute_trajectory_dimensions(tcp, i_v, i_phi, path, true);
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
							//	                        	printf("dist = %lf, i_phi = %lf, k2 = %lf, k3 = %lf, i_v = %lf\n",
							//	                        			get_distance_by_index(dist), get_i_phi_by_index(i_phi), k2, k3, get_initial_velocity_by_index(i_v));
							if (ntcp.valid && has_valid_discretization(tdd) && !path_has_loop(get_distance_by_index(tdd.dist), ntcp.sf))
							{
								TrajectoryLookupTable::TrajectoryControlParameters ptcp = trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i];
								if (!ptcp.valid || (ntcp.sf < ptcp.sf))
								{
									trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i] = ntcp;
									valids += 1.0;
//									printf("TDDV_I: %d, TD_VI: %lf a: %lf,  VF: %lf, TDDTheta: %d TDTheta: %lf\n",tdd.v_i, td.v_i, ntcp.a, ntcp.vf, tdd.theta, td.theta);

									//	                                vector<carmen_ackerman_path_point_t> path;
									//	                                compute_trajectory_dimensions(trajectory_lookup_table[tdd.dist][tdd.theta][tdd.d_yaw][tdd.phi_i][tdd.v_i], i_v, i_phi, path, true);
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

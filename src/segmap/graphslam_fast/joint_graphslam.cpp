
#include <carmen/carmen.h>
#include <iostream>
#include <vector>
#include <cmath>

#include "g2o/types/slam2d/se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/types_slam2d.h"

#include "g2o/apps/g2o_cli/dl_wrapper.h"
#include "g2o/apps/g2o_cli/g2o_common.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/stuff/macros.h"
#include "g2o/core/optimizable_graph.h"

#include "edge_gps_2D.h"

#include "graphslam_util.h"
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_loop_closures.h>

using namespace std;
using namespace g2o;


int
add_connections_between_pair_logs(
	DatasetCarmen &di, DatasetCarmen &dj, 
	vector<int> &di_vertices, vector<int> &dj_vertices,
	SparseOptimizer &optimizer)
{
	int k, l, n_edges;

    Matrix3d interlog_information = create_information_matrix(
		interlog_xy_std, interlog_xy_std, interlog_th_std);

	n_edges = 0;

	for (k = 0; k < di.data.size(); k++)
	{
		for (l = 0; l < dj.data.size(); l++)
		{
			double dth = fabs(carmen_normalize_theta(
				di.data[k].gps.th - dj.data[l].gps.th));

			double d = sqrt(pow(di.data[k].gps.x - dj.data[l].gps.x, 2) + 
				pow(di.data[k].gps.y - dj.data[l].gps.y, 2));
			
			if (fabs(di.data[k].v) > min_vel_to_add_gps && 
				fabs(dj.data[l].v) > min_vel_to_add_gps && 
				dth < carmen_degrees_to_radians(30.) && d < 1.) 
				// && di.data[k].quality == 4 && dj.data[l].quality == 4)
			{
				SE2 gps_i = SE2(di.data[k].gps.x, di.data[k].gps.y, di.data[k].gps.th);
				SE2 gps_j = SE2(dj.data[l].gps.x, dj.data[l].gps.y, dj.data[l].gps.th);
				SE2 measure = gps_i.inverse() * gps_j;

				add_relative_edge(di_vertices[k], dj_vertices[l],
					measure, interlog_information, optimizer);

				n_edges++;
			}
		}
	}

	return n_edges;
}


void
add_interlog_connections(vector<DatasetCarmen> &input_data, 
	vector<vector<int>> &vertices_ids,
	SparseOptimizer &optimizer)
{
	int i, j, n;

	n = 0;

	for (i = 0; i < input_data.size(); i++)
	{
		for (j = i + 1; j < input_data.size(); j++)
		{
			DatasetCarmen di = input_data[i];
			DatasetCarmen dj = input_data[j];

			n += add_connections_between_pair_logs(
				di, dj, 
				vertices_ids[i], 
				vertices_ids[j], 
				optimizer);
		}
	}

	printf("N interlog connections: %d\n", n);
}


void
create_graph(vector<DatasetCarmen> &input_data, 
	vector<vector<int>> &vertices_ids, SparseOptimizer &optimizer)
{
	uint i, j, vertex_id;

	vertex_id = 0;

	Matrix3d odom_information = create_information_matrix(
		odom_xy_std, odom_xy_std, odom_th_std);

	for (i = 0; i < input_data.size(); i++)
	{
		vector<SE2> dead_reckoning;
		create_dead_reckoning(input_data[i], dead_reckoning);
		vertices_ids.push_back(vector<int>());

		for (j = 0; j < input_data[i].data.size(); j++)
		{
			add_vertex(vertex_id, dead_reckoning[j], optimizer);
			vertices_ids[i].push_back(vertex_id);

			if (fabs(input_data[i].data[j].v) > min_vel_to_add_gps) // && input_data[i].data[j].quality == 4)
			{
				double gps_std = gps_std_from_quality(input_data[i].data[j].gps_quality);
				Matrix3d information = create_information_matrix(
					gps_std * gps_xy_std, 
					gps_std * gps_xy_std, 
					gps_th_std);

				SE2 gps_measure(
					input_data[i].data[j].gps.x - input_data[0].data[0].gps.x,
					input_data[i].data[j].gps.y - input_data[0].data[0].gps.y,
					input_data[i].data[j].gps.th);

				add_global_edge(vertex_id, gps_measure, information, optimizer);
			}

			if (j > 0)
			{
				add_relative_edge(
					vertex_id - 1, vertex_id, 
					dead_reckoning[j - 1].inverse() * dead_reckoning[j], 
					odom_information,
					optimizer);
			}

			vertex_id++;
		}
	}

	printf("Num vertices: %d\n", vertex_id);
	add_interlog_connections(input_data, vertices_ids, optimizer);
}


void
save_corrected_vertices_from_datasets(
	vector<DatasetCarmen> &data, 
	vector<vector<int>> &vertices_ids, 
	SparseOptimizer *optimizer)
{
	string cmd = "cat ";

	for (int i = 0; i < data.size(); i++)
	{
		save_corrected_vertices(
			data[i], vertices_ids[i], 
			optimizer,  "optimized_jointly.txt", 
			data[0].data[0].gps.x, data[0].data[0].gps.y);

		cmd += data[i]._path + "/optimized_jointly.txt ";
	}

	string debug_name = data[0]._path + "/optimized_debug.txt";
	cmd += " > " + debug_name;
	system(cmd.c_str());
	printf("Debug file name: %s\n", debug_name.c_str());
}


int 
main(int argc, char **argv)
{
	srand(time(NULL));

	if (argc < 2)
		exit(printf("Use %s <data-directories>\n", argv[0]));

	SparseOptimizer* optimizer;
	DlWrapper dlSolverWrapper;
	// Why does it only works at main?
	loadStandardSolver(dlSolverWrapper, argc, argv);
	Factory* factory = Factory::instance();
	factory->registerType("EDGE_GPS", new HyperGraphElementCreator<EdgeGPS>);
	optimizer = initialize_optimizer();

	vector<DatasetCarmen> data;
	vector<vector<int>> vertices_ids;
	
	for (int i = 1; i < argc; i++)
		data.push_back(DatasetCarmen(argv[i], 0));

	create_graph(data, vertices_ids, *optimizer);
	run_optimization(optimizer);

	save_corrected_vertices_from_datasets(data, vertices_ids, optimizer);
	cerr << "OutputSaved!" << endl;

	return 0;
}

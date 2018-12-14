
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

using namespace std;
using namespace g2o;

const double distance_between_front_and_rear_axles = 2.625;
const double interlog_xy_std = 5.0;
const double interlog_th_std = M_PI * 1e10;
const double odom_xy_std = 0.01;
const double odom_th_std = 0.001;
const double gps_xy_std = 0.2;
const double gps_th_std = M_PI * 1e10;


class SyncDataSample
{
public:
	char cloud_path[256];
	int n_rays;
	double cloud_time;

	char image_path[256];
	int w, h, size, n;
	double image_time;

	double x, y, quality, gps_time;
	double angle, angle_quality, gps2_time;

	double v, phi, ack_time;
};


class OdomData
{
public:
	double mult_v, mult_phi, add_phi, init_angle;
};


class Data
{
public:
	string dirname;
	vector<SyncDataSample> sync;
	vector<SE2> dead_reckoning;
	vector<int> vertices_ids;
	OdomData odom;

	void create_dead_reckoning()
	{
		double x, y, ds, th, dt, v, phi;

		x = y = th = 0.;
		th = odom.init_angle;
		dead_reckoning.push_back(SE2(x, y, th));

		for (size_t i = 1; i < sync.size(); i++)
		{
			dt = sync[i].ack_time - sync[i - 1].ack_time;
			v = sync[i - 1].v * odom.mult_v;
			phi = sync[i - 1].phi * odom.mult_phi + odom.add_phi;

			ds = v * dt;
			x += ds * cos(th);
			y += ds * sin(th);
			th += ds * tan(phi) / distance_between_front_and_rear_axles;
			th = carmen_normalize_theta(th);

			dead_reckoning.push_back(SE2(x, y, th));
		}
	}

	void load_odom_calib()
	{
		string name = dirname + "/odom_calib_stderr.txt";
		FILE *f = fopen(name.c_str(), "r");

		if (f == NULL)
			exit(printf("Error: file '%s' not found.\n", name.c_str()));

		char dummy[64];

		fscanf(f, "%s %s %lf %s %s %s %lf %lf %s %s %lf",
			dummy, dummy, &odom.mult_v, dummy, dummy, dummy, 
			&odom.mult_phi, &odom.add_phi, dummy, dummy, 
			&odom.init_angle);

        odom.mult_v = 1.0;
        odom.add_phi = 0.0;

		printf("Odom data: %lf %lf %lf %lf\n", 
			odom.mult_v, odom.mult_phi, odom.add_phi, odom.init_angle);
	}

	void load_sync()
	{
		string name = dirname + "/sync.txt";
		FILE *f = fopen(name.c_str(), "r");

		if (f == NULL)
			exit(printf("Error: file '%s' not found.\n", name.c_str()));

		char dummy[256];
		int idummy;

		while (!feof(f))
		{
			SyncDataSample d;

			char c = fgetc(f);
			if (c != 'V')
			{
				printf("skipping char %c at line %ld\n", c, sync.size()+1);
				continue;
			}

			fscanf(f, "\n%s %s %d %lf ",
					dummy, d.cloud_path, &d.n_rays, &d.cloud_time);

			fscanf(f, " %s %s %d %d %d %d %lf ",
					dummy, d.image_path, &d.w, &d.h, &d.size, &d.n, &d.image_time);

			fscanf(f, " %s %d %lf %lf %lf %lf ",
				dummy, &idummy, &d.x, &d.y, &d.quality, &d.gps_time);

			fscanf(f, " %s %d %lf %lf %lf ",
				dummy,  &idummy, &d.angle, &d.angle_quality, &d.gps2_time);

			fscanf(f, " %s %lf %lf %lf\n",
				dummy, &d.v, &d.phi, &d.ack_time);

			sync.push_back(d);
		}

		printf("N lines in sync file: %ld\n", sync.size());
		fclose(f);
	}

	Data(char *param_dirname)
	{
		dirname = string(param_dirname);

		printf("Loading data from '%s'\n", param_dirname);

		load_sync();
		load_odom_calib();
		create_dead_reckoning();

		printf("Load done.\n");
	}
};


Matrix3d
create_information_matrix(double x_std, double y_std, double z_std)
{
	Matrix3d cov;

	cov(0, 0) = pow(x_std, 2);
	cov(0, 1) = 0;
	cov(0, 2) = 0;
	cov(1, 0) = 0;
	cov(1, 1) = pow(y_std, 2);
	cov(1, 2) = 0;
	cov(2, 0) = 0;
	cov(2, 1) = 0;
	cov(2, 2) = pow(z_std, 2);

	return cov.inverse();
}


double
gps_std_from_quality(int quality)
{
	double gps_std;

	// @Filipe: desvios padrao para cada modo do GPS Trimble.
	// @Filipe: OBS: Multipliquei os stds por 2 no switch abaixo para dar uma folga.
	// 0: DBL_MAX
	// 1: 4.0
	// 2: 1.0
	// 4: 0.1
	// 5: 0.1
	switch (quality)
	{
		case 1:
			gps_std = 8.0;
			break;
		case 2:
			gps_std = 2.0;
			break;
		case 4:
			gps_std = 0.2;
			break;
		case 5:
			gps_std = 1.5;
			break;
		default:
			gps_std = DBL_MAX;
	}

	return gps_std;
}


void
add_connections_between_pair_logs(Data &di, Data &dj, SparseOptimizer &optimizer)
{
	int k, l;

    Matrix3d interlog_information = create_information_matrix(
		interlog_xy_std, interlog_xy_std, interlog_th_std);

	for (k = 0; k < di.sync.size(); k++)
	{
		for (l = 0; l < dj.sync.size(); l++)
		{
			double dth = fabs(carmen_normalize_theta(
				di.sync[k].angle - dj.sync[l].angle));
			double d = sqrt(pow(di.sync[k].x - dj.sync[l].x, 2) + 
				pow(di.sync[k].y - dj.sync[l].y, 2));
			
			if (fabs(di.sync[k].v) > 3. && fabs(dj.sync[l].v) > 3. && dth < carmen_degrees_to_radians(30.) && d < 1.) // && di.sync[k].quality == 4 && dj.sync[l].quality == 4)
			{
				SE2 gps_i = SE2(di.sync[k].x, di.sync[k].y, di.sync[k].angle);
				SE2 gps_j = SE2(dj.sync[l].x, dj.sync[l].y, dj.sync[l].angle);
				SE2 measure = gps_i.inverse() * gps_j;

				EdgeSE2* edge = new EdgeSE2;
				edge->vertices()[0] = optimizer.vertex(di.vertices_ids[k]);
				edge->vertices()[1] = optimizer.vertex(dj.vertices_ids[l]);
				edge->setMeasurement(measure);
				edge->setInformation(interlog_information);
				optimizer.addEdge(edge);
			}
		}
	}
}


void
add_interlog_connections(vector<Data> &input_data, SparseOptimizer &optimizer)
{
	int i, j, n;

	n = 0;

	for (i = 0; i < input_data.size(); i++)
	{
		for (j = i + 1; j < input_data.size(); j++)
		{
			Data di = input_data[i];
			Data dj = input_data[j];

			add_connections_between_pair_logs(di, dj, optimizer);
			n++;
		}
	}

	printf("N interlog connections: %d\n", n);
}


void
add_vertex(int id, SE2 initial_guess, SparseOptimizer &optimizer)
{
	VertexSE2* vertex = new VertexSE2;
	vertex->setId(id);
	vertex->setEstimate(initial_guess);
	optimizer.addVertex(vertex);
}


void
add_gps_edge(int id, vector<Data> &input_data, 
	int log_id, int sample_id, 
	SparseOptimizer &optimizer)
{
	double gps_std = gps_std_from_quality(input_data[log_id].sync[sample_id].quality);
	Matrix3d information = create_information_matrix(
		gps_std * gps_xy_std, 
		gps_std * gps_xy_std, 
		gps_th_std);

	SE2 gps_measure(
		input_data[log_id].sync[sample_id].x - input_data[0].sync[0].x,
		input_data[log_id].sync[sample_id].y - input_data[0].sync[0].y,
		input_data[log_id].sync[sample_id].angle);
		//0.);

	EdgeGPS *edge_gps = new EdgeGPS;
	edge_gps->vertices()[0] = optimizer.vertex(id);
	edge_gps->setMeasurement(gps_measure);
	edge_gps->setInformation(information);
	optimizer.addEdge(edge_gps);
}

void
add_odom_edge(int id, vector<Data> &input_data, 
	int log_id, int sample_id, 
	SparseOptimizer &optimizer)
{
	Matrix3d odom_information = create_information_matrix(
		odom_xy_std, odom_xy_std, odom_th_std);

	SE2 measure = input_data[log_id].dead_reckoning[sample_id - 1].inverse() * input_data[log_id].dead_reckoning[sample_id];
	EdgeSE2* edge = new EdgeSE2;
	edge->vertices()[0] = optimizer.vertex(id - 1);
	edge->vertices()[1] = optimizer.vertex(id);
	edge->setMeasurement(measure);
	edge->setInformation(odom_information);
	optimizer.addEdge(edge);
}


void
create_graph(vector<Data> &input_data, SparseOptimizer &optimizer)
{
	uint i, j, vertex_id;

	vertex_id = 0;

	for (i = 0; i < input_data.size(); i++)
	{
		for (j = 0; j < input_data[i].sync.size(); j++)
		{
			add_vertex(vertex_id, input_data[i].dead_reckoning[j], optimizer);
			input_data[i].vertices_ids.push_back(vertex_id);

			if (fabs(input_data[i].sync[j].v) > 3.) // && input_data[i].sync[j].quality == 4)
				add_gps_edge(vertex_id, input_data, i, j, optimizer);

			if (j > 0)
				add_odom_edge(vertex_id, input_data, i, j, optimizer);

			vertex_id++;
		}
	}

	printf("Num vertices: %d\n", vertex_id);

	add_interlog_connections(input_data, optimizer);
	
	optimizer.save("poses_before.g2o");
	cout << "load complete!" << endl;
}


void
save_corrected_vertices(vector<Data> &data, SparseOptimizer *optimizer)
{
	string debug_name = data[0].dirname + "/optimized_debug.txt";
	FILE *debug_f = fopen(debug_name.c_str(), "w");

	if (debug_f == NULL)
		exit(printf("File '%s' not created.\n", debug_name.c_str()));

	printf("Debug file: '%s'\n", debug_name.c_str());

	for (int i = 0; i < data.size(); i++)
	{
		Data d = data[i];
		string name = d.dirname + "/optimized_jointly.txt";
		FILE *f = fopen(name.c_str(), "w");

		if (f == NULL)
			exit(printf("File '%s' couldn't be opened!\n", name.c_str()));

		double x, y, th, t;

		for (size_t j = 0; j < d.vertices_ids.size(); j++)
		{
			VertexSE2* v = dynamic_cast<VertexSE2*>(optimizer->vertex(d.vertices_ids[j]));
			SE2 pose = v->estimate();

			x = pose.toVector().data()[0] + data[0].sync[0].x;
			y = pose.toVector().data()[1] + data[0].sync[0].y;
			th = pose.toVector().data()[2];
			t = d.sync[j].cloud_time;

			fprintf(f, "%ld %lf %lf %lf %lf %lf %lf %lf %lf\n", 
				j, x, y, th, t, 
				d.sync[j].image_time, d.sync[j].v,
				d.sync[j].phi, d.sync[j].ack_time);

			fprintf(debug_f, "%lf %lf %lf %lf\n", 
				x - data[0].sync[0].x, 
				y - data[0].sync[0].y, 
				d.sync[j].x - data[0].sync[0].x, 
				d.sync[j].y - data[0].sync[0].y);
		}

		fclose(f);
	}

	fclose(debug_f);
}


void
prepare_optimization(SparseOptimizer *optimizer)
{
	for (SparseOptimizer::VertexIDMap::const_iterator it = optimizer->vertices().begin(); it != optimizer->vertices().end(); ++it)
	{
		OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
		v->setMarginalized(false);
	}

	optimizer->initializeOptimization();

	for (SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it)
	{
		OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
		e->setRobustKernel(0);
	}
}


SparseOptimizer*
initialize_optimizer()
{
	SparseOptimizer *optimizer = new SparseOptimizer;

	OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
	g2o::OptimizationAlgorithmProperty _currentOptimizationAlgorithmProperty;
	// _currentOptimizationAlgorithmProperty.name = "lm_pcg";
	_currentOptimizationAlgorithmProperty.name = "gn_var_cholmod";
	_currentOptimizationAlgorithmProperty.requiresMarginalize = false;
	// _currentOptimizationAlgorithmProperty.type = "PCG";
	_currentOptimizationAlgorithmProperty.type = "CHOLMOD";
	_currentOptimizationAlgorithmProperty.landmarkDim = -1;
	_currentOptimizationAlgorithmProperty.poseDim = -1;
	_currentOptimizationAlgorithmProperty.desc = ""; // "Gauss-Newton: Cholesky solver using CHOLMOD (variable blocksize)";

	// OptimizationAlgorithm *solver = solverFactory->construct("lm_pcg", _currentOptimizationAlgorithmProperty);
	OptimizationAlgorithm *solver = solverFactory->construct("gn_var_cholmod", _currentOptimizationAlgorithmProperty);

	optimizer->setAlgorithm(solver);

	return optimizer;
}


void
run_optimization(SparseOptimizer* optimizer)
{
	optimizer->setVerbose(true);
	cerr << "Optimizing" << endl;
	prepare_optimization(optimizer);
	optimizer->optimize(20);
	cerr << "OptimizationDone!" << endl;
}


int 
main(int argc, char **argv)
{
	srand(time(NULL));

	if (argc < 2)
		exit(printf("Use %s <data-directories>\n", argv[0]));

	vector<Data> data;
	for (int i = 1; i < argc; i++)
		data.push_back(Data(argv[i]));

	SparseOptimizer* optimizer;
	DlWrapper dlSolverWrapper;
	loadStandardSolver(dlSolverWrapper, argc, argv);
	Factory* factory = Factory::instance();
	factory->registerType("EDGE_GPS", new HyperGraphElementCreator<EdgeGPS>);
	optimizer = initialize_optimizer();

	create_graph(data, *optimizer);
	run_optimization(optimizer);

	save_corrected_vertices(data, optimizer);
	cerr << "OutputSaved!" << endl;

	return 0;
}


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

class Data
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


class LoopRestriction
{
	public:
		SE2 transform;
		int converged;
		int from;
		int to;
};


const double distance_between_front_and_rear_axles = 2.625;


void
load_data(char *name, vector<Data> &lines)
{
	FILE *f = fopen(name, "r");
	char dummy[256];
	int idummy;

	while (!feof(f))
	{
		Data d;

/*
VELODYNE_PARTIAL_SCAN_IN_FILE /dados/logs/log_volta_da_ufes-20180907-2.txt_velodyne/1536320000/1536325900/1536325965.083151.pointcloud 1085 1536325965.083151
BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3 /dados/logs/log_volta_da_ufes-20180907-2.txt_bumblebee/1536320000/1536325900/1536325965.095084.bb3.image 1280 960
3686400 1 1536325965.095084 GPS_XYZ 1 7757734.713280 -363558.145922 4 1536325965.087092 NMEAHDT 1 0.656872 1 1536325965.087092 ROBOTVELOCITY_ACK -0.001000
0.000440 1536325965.073174
*/

		char c = fgetc(f);
		if (c != 'V')
		{
			printf("skipping char %c at line %ld\n", c, lines.size()+1);
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

		lines.push_back(d);
	}

	fclose(f);
}


void
read_loop_restrictions(char *filename, vector<LoopRestriction> &loop_data)
{
	int n;
	FILE *f;
	double x, y, theta;

	if ((f = fopen(filename, "r")) == NULL)
		exit(printf("Error: Unable to open file '%s'!\n", filename));

	while(!feof(f))
	{
		LoopRestriction l;

		n = fscanf(f, "%d %d %d %lf %lf %lf\n",
			&l.from, &l.to, &l.converged, &x, &y, &theta
		);

		if (n == 6)
		{
			l.transform = SE2(x, y, theta);
			loop_data.push_back(l);
		}
	}

	fclose(f);
}


void
add_vertices(vector<Data> &input_data, SparseOptimizer *optimizer)
{
	uint i;

	for (i = 0; i < input_data.size(); i++)
	{
		SE2 estimate(
			input_data[i].x - input_data[0].x,
			input_data[i].y - input_data[0].y,
			input_data[i].angle
		);

		VertexSE2* vertex = new VertexSE2;
		vertex->setId(i);
		vertex->setEstimate(estimate);
		optimizer->addVertex(vertex);
	}
}


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


void
add_odometry_edges(vector<Data> &input_data, SparseOptimizer *optimizer, vector<SE2> &dead_reckoning, double xy_std, double th_std)
{
	double dt;
	Matrix3d information = create_information_matrix(xy_std, xy_std, th_std);

	for (size_t i = 1; i < input_data.size(); i++)
	{
		dt = input_data[i].ack_time - input_data[i - 1].ack_time;

		if (dt > 0 && dt < 600)  // we are in the same log
		{
			SE2 measure = dead_reckoning[i - 1].inverse() * dead_reckoning[i];
			EdgeSE2* edge = new EdgeSE2;
			edge->vertices()[0] = optimizer->vertex(i - 1);
			edge->vertices()[1] = optimizer->vertex(i);
			edge->setMeasurement(measure);
			edge->setInformation(information);
			optimizer->addEdge(edge);
		}
	}
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
add_gps_edges(vector<Data> &input_data, SparseOptimizer *optimizer)
{
	for (size_t i = 0; i < input_data.size(); i++)
	{
		SE2 measure(input_data[i].x - input_data[0].x,
					input_data[i].y - input_data[0].y,
					input_data[i].angle);

		double gps_std = gps_std_from_quality(input_data[i].quality);
		Matrix3d information = create_information_matrix(gps_std * 70., gps_std * 70., 3.14 * 1000000);

		EdgeGPS *edge_gps = new EdgeGPS;
		edge_gps->vertices()[0] = optimizer->vertex(i);
		edge_gps->setMeasurement(measure);
		edge_gps->setInformation(information);
		optimizer->addEdge(edge_gps);
	}
}


void
add_loop_closure_edges(vector<LoopRestriction> &loop_data, SparseOptimizer *optimizer)
{
	Matrix3d information = create_information_matrix(50., 50., 0.6);

	for (size_t i = 0; i < loop_data.size(); i++)
	{
		if (loop_data[i].converged)
		{
			EdgeSE2* edge = new EdgeSE2;
			edge->vertices()[0] = optimizer->vertex(loop_data[i].from);
			edge->vertices()[1] = optimizer->vertex(loop_data[i].to);
			edge->setMeasurement(loop_data[i].transform);
			edge->setInformation(information);
			optimizer->addEdge(edge);
		}
	}
}


void
create_dead_reckoning(vector<Data> &input_data, vector<SE2> &dead_reckoning)
{
	double x, y, ds, th, dt, v, phi;

	x = y = th = 0.;
	dead_reckoning.push_back(SE2(x, y, th));

	for (size_t i = 1; i < input_data.size(); i++)
	{
		dt = input_data[i].ack_time - input_data[i - 1].ack_time;

		if (dt < 0 || dt > 600)  // a new log started
		{
			printf("Starting a new log at message %ld from %ld\n", i, input_data.size());
			x = y = th = 0.;
			dead_reckoning.push_back(SE2(x, y, th));
			continue;
		}

		/*
		bias v: 1.006842 -0.000000 bias phi: 0.861957 -0.002372
		Fitness (MSE): -22.967871
		Fitness (SQRT(MSE)): 4.792481
		Initial angle: 0.641244
		*/
		v = input_data[i - 1].v * 1.006842;
		phi = input_data[i - 1].phi * 0.861957 - 0.002372;

		ds = v * dt;
		x += ds * cos(th);
		y += ds * sin(th);
		th += ds * tan(phi) / distance_between_front_and_rear_axles;
		th = carmen_normalize_theta(th);

		dead_reckoning.push_back(SE2(x, y, th));
	}
}


void
load_data_to_optimizer(vector<Data> &input_data, vector<LoopRestriction> &loop_data, SparseOptimizer *optimizer)
{
	vector<SE2> dead_reckoning;

	add_vertices(input_data, optimizer);
	create_dead_reckoning(input_data, dead_reckoning);
	add_odometry_edges(input_data, optimizer, dead_reckoning, 0.1, 0.009);
	add_gps_edges(input_data, optimizer);
	//add_loop_closure_edges(loop_data, optimizer);

	optimizer->save("poses_before.g2o");
	cout << "load complete!" << endl;
}


void
save_corrected_vertices(char *out_name, vector<Data> &input_data, SparseOptimizer *optimizer)
{
	int log_id = 0;
	double x, y, th, t, dt;

	char name_with_index[256];

	sprintf(name_with_index, "%s.%02d.txt", out_name, log_id);
	FILE *f = fopen(name_with_index, "w");

	if (f == NULL)
		exit(printf("File '%s' couldn't be opened!\n", name_with_index));

	int start = -1;

	for (size_t i = 0; i < optimizer->vertices().size(); i++)
	{
		if (i > 0)
		{
			dt = input_data[i].cloud_time - input_data[i - 1].cloud_time;

			if (dt < 0 || dt > 600)
			{
				printf("Initializing a new log at message %ld of %ld\n", i, input_data.size());
				log_id++;

				fclose(f);
				sprintf(name_with_index, "%s.%02d.txt", out_name, log_id);
				f = fopen(name_with_index, "w");

				if (f == NULL)
					exit(printf("File '%s' couldn't be opened!\n", name_with_index));
			}
		}

		VertexSE2* v = dynamic_cast<VertexSE2*>(optimizer->vertex(i));
		SE2 pose = v->estimate();

		x = pose.toVector().data()[0] + input_data[0].x;
		y = pose.toVector().data()[1] + input_data[0].y;
		th = pose.toVector().data()[2];
		t = input_data[i].cloud_time;

		//if (input_data[i].v > 1.0)
		{
			fprintf(f, "%d %lf %lf %lf %lf %lf %lf %lf %lf\n", i, x, y, th, t, input_data[i].image_time, input_data[i].v,
					input_data[i].phi, input_data[i].ack_time);
			if (start == -1) start = i;
		}

		//if (start != -1 && i - start > 1500)
			//break;
	}

	fclose(f);
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


int main(int argc, char **argv)
{
	if (argc < 4)
	{
		exit(printf("Use %s <sync-file> <loops-file> <saida.txt>\n", argv[0]));
	}

	vector<Data> input_data;
	vector<LoopRestriction> loop_data;

	SparseOptimizer* optimizer;

	srand(time(NULL));

	DlWrapper dlSolverWrapper;
	loadStandardSolver(dlSolverWrapper, argc, argv);
	Factory* factory = Factory::instance();
	factory->registerType("EDGE_GPS", new HyperGraphElementCreator<EdgeGPS>);

	optimizer = initialize_optimizer();

	load_data(argv[1], input_data);
	read_loop_restrictions(argv[2], loop_data);
	load_data_to_optimizer(input_data, loop_data, optimizer);

	optimizer->setVerbose(true);
	cerr << "Optimizing" << endl;
	prepare_optimization(optimizer);
	optimizer->optimize(20);
	cerr << "OptimizationDone!" << endl;
	save_corrected_vertices(argv[3], input_data, optimizer);
	cerr << "OutputSaved!" << endl;

	return 0;
}




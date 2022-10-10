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

#define _SEED_RAND 1

/**
 * TODO: Estas estruturas estao replicadas aqui e no graphslam. Colocar em um arquivo .h
 */
class Line
{
	public:
		SE2 pose;
		double time;
};

class LoopRestriction
{
	public:
		SE2 transform;
		int from;
		int to;
};

vector<Line> input_data_1;
vector<Line> input_data_2;
vector<LoopRestriction> loop_data;

char *input_file_1;
char *input_file_2;
char *loops_file;
char *out_file;

void
read_data(char *filename, vector<Line> &data)
{
	int n;
	FILE *f;
	double x, y, theta;
	double time;

	if ((f = fopen(filename, "r")) == NULL)
		exit(printf("Error: Unable to open file '%s'!\n", filename));

	int is_first = 1;
	double dist = 0;

	while(!feof(f))
	{
		n = fscanf(f, "%lf %lf %lf %lf\n", &x, &y, &theta, &time);

		if (n == 4)
		{
			Line l;

			l.pose = SE2(x, y, theta);
			l.time = time;

			data.push_back(l);

			if (!is_first)
			{
				SE2 t = data[data.size() - 2].pose.inverse() * data[data.size() - 1].pose;
				dist += sqrt(pow(t[0], 2) + pow(t[1], 2));
			}

			is_first = 0;
		}
	}

	printf("dist: %lf\n", dist);
	fclose(f);
}


void
read_loop_restrictions(char *filename, vector<LoopRestriction> &data)
{
	int n;
	FILE *f;
	double x, y, theta;

	if ((f = fopen(filename, "r")) == NULL)
		exit(printf("Error: Unable to open file '%s'!\n", filename));

	while(!feof(f))
	{
		LoopRestriction l;

		n = fscanf(f, "%d %d %lf %lf %lf\n",
			&l.from, &l.to, &x, &y, &theta
		);

		if (n == 5)
		{
			l.transform = SE2(x, y, theta);
			data.push_back(l);
		}
	}

	fclose(f);
}


double
r2d(double angle)
{
	return (angle * 180.0) / M_PI;
}


void
add_vertices(SparseOptimizer *optimizer)
{
	uint i;

	/// **********************************
	/// INPUT DATA 1 (FIXED)
	/// **********************************

	for (i = 0; i < input_data_1.size(); i++)
	{
		SE2 estimate(
			input_data_1[i].pose.toVector()[0] - input_data_1[0].pose.toVector()[0],
			input_data_1[i].pose.toVector()[1] - input_data_1[0].pose.toVector()[1],
			input_data_1[i].pose.toVector()[2]
		);

		VertexSE2* vertex = new VertexSE2;
		vertex->setId(i);
		vertex->setEstimate(estimate);
		vertex->setFixed(true);
		optimizer->addVertex(vertex);
	}

	/// **********************************
	/// INPUT DATA 2 (VARIABLE)
	/// **********************************

	for (i = 0; i < input_data_2.size(); i++)
	{
		SE2 estimate(
			input_data_2[i].pose.toVector()[0] - input_data_1[0].pose.toVector()[0],
			input_data_2[i].pose.toVector()[1] - input_data_1[0].pose.toVector()[1],
			input_data_2[i].pose.toVector()[2]
		);

		VertexSE2* vertex = new VertexSE2;
		vertex->setId(i + input_data_1.size());
		vertex->setEstimate(estimate);
		optimizer->addVertex(vertex);
	}
}


void
add_odometry_edges(SparseOptimizer *optimizer)
{
	Matrix3d cov;
	Matrix3d information;

	cov.data()[0] = pow(0.01, 2);
	cov.data()[1] = 0;
	cov.data()[2] = 0;
	cov.data()[3] = 0;
	cov.data()[4] = pow(0.01, 2);
	cov.data()[5] = 0;
	cov.data()[6] = 0;
	cov.data()[7] = 0;
	cov.data()[8] = pow(0.01, 2);

	information = cov.inverse();

	for (size_t i = 0; i < (input_data_1.size() - 1); i++)
	{
		SE2 measure = input_data_1[i].pose.inverse() * input_data_1[i + 1].pose;
		EdgeSE2* edge = new EdgeSE2;
		edge->vertices()[0] = optimizer->vertex(i);
		edge->vertices()[1] = optimizer->vertex(i + 1);
		edge->setMeasurement(measure);
		edge->setInformation(information);
		optimizer->addEdge(edge);
	}

	for (size_t i = 0; i < (input_data_2.size() - 1); i++)
	{
		SE2 measure = input_data_2[i].pose.inverse() * input_data_2[i + 1].pose;

		double dist = sqrt(pow(measure[0], 2) + pow(measure[1], 2));
		double newx = 1.005 * dist * cos(measure[2]);
		double newy = 1.005 * dist * sin(measure[2]);
		measure.setTranslation(Vector2d(newx, newy));

		EdgeSE2* edge = new EdgeSE2;
		edge->vertices()[0] = optimizer->vertex(input_data_1.size() + i);
		edge->vertices()[1] = optimizer->vertex(input_data_1.size() + i + 1);
		edge->setMeasurement(measure);
		edge->setInformation(information);
		optimizer->addEdge(edge);
	}
}


void
add_loop_closure_edges(SparseOptimizer *optimizer)
{
	Matrix3d cov;
	Matrix3d information;

	cov.data()[0] = pow(10.0, 2);
	cov.data()[1] = 0;
	cov.data()[2] = 0;
	cov.data()[3] = 0;
	cov.data()[4] = pow(10.0, 2);
	cov.data()[5] = 0;
	cov.data()[6] = 0;
	cov.data()[7] = 0;
	cov.data()[8] = pow(0.6, 2);

	information = cov.inverse();

	for (size_t i = 0; i < loop_data.size(); i++)
	{
		EdgeSE2* edge = new EdgeSE2;
		edge->vertices()[0] = optimizer->vertex(loop_data[i].from);
		edge->vertices()[1] = optimizer->vertex(loop_data[i].to + input_data_1.size());
		edge->setMeasurement(loop_data[i].transform);
		edge->setInformation(information);
		optimizer->addEdge(edge);
	}
}


void
load_data_to_optimizer(SparseOptimizer *optimizer)
{
	read_data(input_file_1, input_data_1);
	read_data(input_file_2, input_data_2);
	read_loop_restrictions(loops_file, loop_data);

	add_vertices(optimizer);
	add_odometry_edges(optimizer);
	add_loop_closure_edges(optimizer);

	optimizer->save("poses_before.g2o");
	cout << "load complete!" << endl;
}


void
save_corrected_vertices(SparseOptimizer *optimizer)
{
	double *vertex_data;

	FILE *f = fopen(out_file, "w");

	for (size_t i = 0; i < input_data_1.size(); i++)
	{
		VertexSE2* v = dynamic_cast<VertexSE2*>(optimizer->vertex(i));
		SE2 pose = v->estimate();
		pose.setTranslation(Vector2d(v->estimate()[0] + input_data_1[0].pose[0], v->estimate()[1] + input_data_1[0].pose[1]));
		pose.setRotation(Rotation2Dd(v->estimate()[2]));
		vertex_data = pose.toVector().data();
		fprintf(f, "%lf %lf %lf %lf\n", vertex_data[0], vertex_data[1], vertex_data[2], input_data_1[i].time);
	}

	for (size_t i = 0; i < input_data_2.size(); i++)
	{
		VertexSE2* v = dynamic_cast<VertexSE2*>(optimizer->vertex(i + input_data_1.size()));
		SE2 pose = v->estimate();
		pose.setTranslation(Vector2d(v->estimate()[0] + input_data_1[0].pose[0], v->estimate()[1] + input_data_1[0].pose[1]));
		pose.setRotation(Rotation2Dd(v->estimate()[2]));
		vertex_data = pose.toVector().data();
		fprintf(f, "%lf %lf %lf %lf\n", vertex_data[0], vertex_data[1], vertex_data[2], input_data_2[i].time);
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
	if (argc < 5)
	{
		exit(printf("Use %s <input-file-1> <input-file-2> <loops-file> <saida.txt>\n", argv[0]));
	}

	input_file_1 = argv[1];
	input_file_2 = argv[2];
	loops_file = argv[3];
	out_file = argv[4];

	SparseOptimizer* optimizer;

	srand(time(NULL));
	if (_SEED_RAND)
		srand(42);

	DlWrapper dlSolverWrapper;
	loadStandardSolver(dlSolverWrapper, argc, argv);
	Factory* factory = Factory::instance();
	factory->registerType("EDGE_GPS", new HyperGraphElementCreator<EdgeGPS>);

	optimizer = initialize_optimizer();
	load_data_to_optimizer(optimizer);

	optimizer->setVerbose(true);
	cerr << "Optimizing" << endl;
	prepare_optimization(optimizer);
	optimizer->optimize(20);
	cerr << "OptimizationDone!" << endl;
	save_corrected_vertices(optimizer);
	cerr << "OutputSaved!" << endl;

	printf("Tecle ctrl+c para terminar.\n");
	fflush(stdout);
	getchar();

	return 0;
}

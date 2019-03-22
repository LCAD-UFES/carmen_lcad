
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_util.h>
#include <carmen/carmen.h>
#include <iostream>
#include <string>
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
#include <carmen/gicp.h>
#include <carmen/segmap_command_line.h>

using namespace std;
using namespace g2o;


class GraphSlamData
{
public:
	NewCarmenDataset *dataset;
	vector<LoopRestriction> loop_data, gicp_odom_data, gicp_fake_gps;

	~GraphSlamData() { delete(dataset);	}

	string log_file;
	string loop_closure_file;
	string gicp_odom_file;
	string gicp_map_file;
	string output_file;
	string odom_calib_file;

	double gps_xy_std, gps_angle_std;
	double odom_xy_std, odom_angle_std;
	double loop_xy_std, loop_angle_std;
	double gicp_to_map_xy_std, gicp_to_map_angle_std;
	double gicp_odom_xy_std, gicp_odom_angle_std;

	int n_iterations;

	double min_velocity_for_estimating_heading;
	double min_velocity_for_considering_gps;
	string global_angle_mode;

protected:
	void _load_parameters(char *config);
};


const double distance_between_front_and_rear_axles = 2.625;


void
read_loop_restrictions(string &filename, vector<LoopRestriction> *loop_data)
{
	int n;
	FILE *f;
	double x, y, theta;

	if (filename.size() <= 0)
		return;

	f = fopen(filename.c_str(), "r");

	if (f != NULL)
	{
		while(!feof(f))
		{
			LoopRestriction l;

			n = fscanf(f, "%d %d %d %lf %lf %lf\n",
								 &l.from, &l.to, &l.converged, &x, &y, &theta
			);

			if (n == 6)
			{
				l.transform = SE2(x, y, theta);
				loop_data->push_back(l);
			}
		}

		fclose(f);
	}
	else
		fprintf(stderr, "Warning: ignoring loop closure file '%s'\n", filename.c_str());
}


void
add_vertices(vector<SE2> &dead_reckoning, SparseOptimizer *optimizer)
{
	uint i;

	for (i = 0; i < dead_reckoning.size(); i++)
	{
		//SE2 estimate(
		//	input_data[i].x - input_data[0].x,
		//	input_data[i].y - input_data[0].y,
		//	input_data[i].angle
		//);

		SE2 estimate = dead_reckoning[i];
		//SE2 estimate = SE2(i, 0, 0);

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
add_odometry_edges(SparseOptimizer *optimizer, vector<SE2> &dead_reckoning, double xy_std, double th_std)
{
	Matrix3d information = create_information_matrix(xy_std, xy_std, th_std);

	for (size_t i = 1; i < dead_reckoning.size(); i++)
	{
		SE2 prev_inv = dead_reckoning[i - 1].inverse();
		SE2 measure = prev_inv * dead_reckoning[i];

//		printf("i: %lf %lf %lf prev_inv: %lf %lf %lf measure: %lf %lf %lf\n",
//					 dead_reckoning[i][0],dead_reckoning[i][1], dead_reckoning[i][2],
//					 prev_inv[0], prev_inv[1],prev_inv[2],
//					 measure[0], measure[1], measure[2]);

		EdgeSE2* edge = new EdgeSE2;
		edge->vertices()[0] = optimizer->vertex(i - 1);
		edge->vertices()[1] = optimizer->vertex(i);
		edge->setMeasurement(measure);
		edge->setInformation(information);
		optimizer->addEdge(edge);
	}
}


void
compute_angle_from_gps(GraphSlamData &data, int i,
                       double th_std,
                       double *angle,
                       double *filtered_th_std)
{

	DataSample *sample = data.dataset->at(i);

	if (data.global_angle_mode.compare("xsens") == 0)
	{
		Matrix<double, 3, 1> ypr = sample->xsens.toRotationMatrix().eulerAngles(2, 1, 0);
		*angle = ypr(0, 0);
		*filtered_th_std = th_std;
	}
	else if (data.global_angle_mode.compare("gps") == 0)
	{
		*angle = sample->gps.th;
		*filtered_th_std = th_std;
	}
	else if (data.global_angle_mode.compare("consecutive_gps") == 0)
	{
		// estimate the angle using consecutive gps positions
		// when the car velocity is high enough.
		if (i > 0 && fabs(sample->v) >= data.min_velocity_for_estimating_heading)
		{
			DataSample *prev = data.dataset->at(i - 1);
			*angle = atan2(sample->gps.y - prev->gps.y, sample->gps.x - prev->gps.x);
			*filtered_th_std = th_std;
		}
		else
		{
			// if the car is slow, ignore the heading and use a high angular variance.
			*angle = 0.;
			*filtered_th_std = 1e8;
		}
	}
	else
		exit(printf("Error: Invalid global angle estimation mode '%s'\n", data.global_angle_mode.c_str()));
}


void
add_gps_edges(GraphSlamData &data, SparseOptimizer *optimizer,
              double xy_std, double th_std)
{
	double angle, filtered_th_std;
	Pose2d gps0;
	DataSample *sample;
	Matrix3d information;

	gps0 = data.dataset->at(0)->gps;

	for (int i = 0; i < data.dataset->size(); i++)
	{
		sample = data.dataset->at(i);

		// ignore the messages when the car is almost stopped.
		if (fabs(sample->v) < data.min_velocity_for_considering_gps)
			continue;

		compute_angle_from_gps(data, i, th_std,
		                       &angle,
		                       &filtered_th_std);

		information = create_information_matrix(xy_std, xy_std, th_std);

		SE2 measure(sample->gps.x - gps0.x,
		            sample->gps.y - gps0.y,
		            angle);

//		printf("GPS: measure: %lf %lf %lf x_std: %lf th_std: %lf\n",
//					 measure[0], measure[1], angle,
//					 xy_std, filtered_th_std);

		EdgeGPS *edge_gps = new EdgeGPS;
		edge_gps->vertices()[0] = optimizer->vertex(i);
		edge_gps->setMeasurement(measure);
		edge_gps->setInformation(information);
		optimizer->addEdge(edge_gps);
	}
}


void
add_gps_gicp_edges(vector<LoopRestriction> &gicp_gps, SparseOptimizer *optimizer, double xy_std_mult, double th_std)
{
	for (size_t i = 0; i < gicp_gps.size(); i += 1)
	{
		if (gicp_gps[i].converged)
		{
			SE2 measure(gicp_gps[i].transform[0] - gicp_gps[0].transform[0],
			            gicp_gps[i].transform[1] - gicp_gps[0].transform[1],
			            gicp_gps[i].transform[2]);

			Matrix3d information = create_information_matrix(xy_std_mult, xy_std_mult, th_std);

			EdgeGPS *edge_gps = new EdgeGPS;
			edge_gps->vertices()[0] = optimizer->vertex(gicp_gps[i].from);
			edge_gps->setMeasurement(measure);
			edge_gps->setInformation(information);
			optimizer->addEdge(edge_gps);
		}
	}
}


void
add_loop_closure_edges(vector<LoopRestriction> &loop_data, SparseOptimizer *optimizer, double xy_std, double th_std)
{
	Matrix3d information = create_information_matrix(xy_std, xy_std, th_std);

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
create_dead_reckoning(GraphSlamData &data, vector<SE2> &dead_reckoning)
{
	double dt;
	DataSample *sample;

	Pose2d pose(0, 0, data.dataset->initial_angle());
	dead_reckoning.push_back(SE2(pose.x, pose.y, pose.th));

	for (int i = 1; i < data.dataset->size(); i++)
	{
		sample = data.dataset->at(i);

		dt = sample->image_time - data.dataset->at(i - 1)->image_time;
		ackerman_motion_model(pose, sample->v, sample->phi, dt);
		//printf("Dead reckoning: %d dt: %lf v: %lf phi: %lf x: %lf y: %lf\n", i, dt, sample->v, sample->phi, pose.x, pose.y);
		dead_reckoning.push_back(SE2(pose.x, pose.y, pose.th));

	}

	printf("N odometry edges: %ld\n", dead_reckoning.size() - 1);
}


void
load_data_to_optimizer(GraphSlamData &data, SparseOptimizer* optimizer)
{
	vector<SE2> dead_reckoning;

	create_dead_reckoning(data, dead_reckoning);
	add_vertices(dead_reckoning, optimizer);
	add_odometry_edges(optimizer, dead_reckoning, data.odom_xy_std, deg2rad(data.odom_angle_std));

	if (data.gicp_fake_gps.size() > 0)
		add_gps_gicp_edges(data.gicp_fake_gps, optimizer, data.gicp_to_map_xy_std, deg2rad(data.gicp_to_map_angle_std));
	else
		add_gps_edges(data, optimizer, data.gps_xy_std, deg2rad(data.gps_angle_std));

	add_loop_closure_edges(data.loop_data, optimizer, data.loop_xy_std, deg2rad(data.loop_angle_std));
	add_loop_closure_edges(data.gicp_odom_data, optimizer, data.gicp_odom_xy_std, deg2rad(data.gicp_odom_angle_std));

}


void
save_corrected_vertices(GraphSlamData &data, SparseOptimizer *optimizer)
{
	double x, y, th;
	DataSample *sample;

	FILE *f = safe_fopen(data.output_file.c_str(), "w");
	Pose2d gps0 = data.dataset->at(0)->gps;

	for (size_t i = 0; i < optimizer->vertices().size(); i++)
	{
		sample = data.dataset->at(i);

		VertexSE2* v = dynamic_cast<VertexSE2*>(optimizer->vertex(i));
		SE2 pose = v->estimate();

		x = pose.toVector().data()[0] + gps0.x;
		y = pose.toVector().data()[1] + gps0.y;
		th = pose.toVector().data()[2];

		fprintf(f, "%ld %lf %lf %lf %lf %lf %lf\n", i, x, y, th, sample->image_time, sample->gps.x, sample->gps.y);
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


void
load_parameters(int argc, char **argv, GraphSlamData *data)
{
	CommandLineArguments args;

	args.add_positional<string>("log", "Path to a log", 1);
	args.add_positional<string>("output", "Path to the output file", 1);
	args.add<string>("odom_calib,o", "Path to the odometry calibration file", "none");
	args.add<string>("loops,l", "Path to a file with loop closure relations", "none");
	args.add<string>("gicp_odom", "Path to a file with odometry relations generated with GICP", "none");
	args.add<string>("gicp_to_map", "Path to a file with poses given by registration to an a priori map", "none");
	args.add<double>("odom_xy_std", "Std in xy of odometry-based movement estimations (m)", 0.005);
	args.add<double>("gps_xy_std", "Std in xy of gps measurements (m)", 10.0);
	args.add<double>("loop_xy_std", "Std in xy of loop closure measurements (m)", 0.3);
	args.add<double>("gicp_odom_xy_std", "Std in xy of odometry estimated using GICP (m)", 0.3);
	args.add<double>("gicp_to_map_xy_std", "Std in xy of loop closure measurements in relation to a map (m)", 0.3);
	args.add<double>("odom_angle_std", "Std of heading of odometry-based movement estimations (degrees)", 0.1);
	args.add<double>("gps_angle_std", "Std of heading estimated using consecutive gps measurements when the car is moving (degrees)", 10);
	args.add<double>("loop_angle_std", "Std of heading in loop closure measurements (degrees)", 3);
	args.add<double>("gicp_odom_angle_std", "Std of heading when estimating odometry using GICP (degrees)", 3);
	args.add<double>("gicp_to_map_angle_std", "Std of heading in loop closure in relation to a map (degrees)", 3);
	args.add<double>("v_for_gps_heading", "Minimum velocity for estimating heading with consecutive GPS poses", 2.0);
	args.add<double>("v_for_gps", "Minimum velocity for adding GPS edges (without this trick, positions in which v=0 can be overweighted)", 2.0);
	args.add<string>("global_angle_mode", "Method for estimating global angle: [xsens, gps, consecutive_gps]", "consecutive_gps");
	args.add<int>("n_iterations,n", "Number of optimization terations", 50);

	args.save_config_file("data/graphslam_config.txt");
	args.parse(argc, argv);

	data->log_file = args.get<string>("log");
	data->output_file = args.get<string>("output");
	data->odom_calib_file = args.get<string>("odom_calib");
	data->loop_closure_file = args.get<string>("loops");
	data->gicp_odom_file = args.get<string>("gicp_odom");
	data->gicp_map_file = args.get<string>("gicp_to_map");
	data->odom_xy_std = args.get<double>("odom_xy_std");
	data->gps_xy_std = args.get<double>("gps_xy_std");
	data->loop_xy_std = args.get<double>("loop_xy_std");
	data->gicp_odom_xy_std = args.get<double>("gicp_odom_xy_std");
	data->gicp_to_map_xy_std = args.get<double>("gicp_to_map_xy_std");
	data->odom_angle_std = args.get<double>("odom_angle_std");
	data->gps_angle_std = args.get<double>("gps_angle_std");
	data->loop_angle_std = args.get<double>("loop_angle_std");
	data->gicp_odom_angle_std = args.get<double>("gicp_odom_angle_std");
	data->gicp_to_map_angle_std = args.get<double>("gicp_to_map_angle_std");
	data->n_iterations = args.get<int>("n_iterations");
	data->min_velocity_for_estimating_heading = args.get<double>("v_for_gps_heading");
	data->min_velocity_for_considering_gps = args.get<double>("v_for_gps");
	data->global_angle_mode = args.get<string>("global_angle_mode");

}


int main(int argc, char **argv)
{
	srand(time(NULL));

	GraphSlamData data;
	load_parameters(argc, argv, &data);

	// initialize optimizer (why does the initialization using the factory only works in the main?).
	SparseOptimizer* optimizer;
	DlWrapper dlSolverWrapper;
	loadStandardSolver(dlSolverWrapper, argc, argv);
	Factory* factory = Factory::instance();
	factory->registerType("EDGE_GPS", new HyperGraphElementCreator<EdgeGPS>);
	optimizer = initialize_optimizer();

	data.dataset = new NewCarmenDataset(data.log_file, data.odom_calib_file);
	read_loop_restrictions(data.loop_closure_file, &data.loop_data);
	read_loop_restrictions(data.gicp_odom_file, &data.gicp_odom_data);
	read_loop_restrictions(data.gicp_map_file, &data.gicp_fake_gps);

	load_data_to_optimizer(data, optimizer);

	optimizer->setVerbose(true);
	prepare_optimization(optimizer);
	optimizer->optimize(data.n_iterations);
	cerr << "OptimizationDone!" << endl;

	// output
	save_corrected_vertices(data, optimizer);
	cerr << "OutputSaved!" << endl;

	return 0;
}


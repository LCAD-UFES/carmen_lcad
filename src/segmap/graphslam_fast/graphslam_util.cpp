
#include <cfloat>
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "edge_gps_2D.h"

#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <carmen/segmap_util.h>
#include "graphslam_util.h"

using namespace g2o;
using namespace Eigen;


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

	return 1.0; // gps_std;
}


void
create_dead_reckoning(DatasetCarmen &dataset, vector<SE2> &dead_reckoning)
{
	double dt, calib_v, calib_phi;
    double x, y, th;

    x = y = 0.;
    th = dataset.odom_calib.init_angle;

    dead_reckoning.push_back(SE2(x, y, th));

	for (int i = 1; i < dataset.data.size(); i++)
	{
		dt = dataset.data[i].odom_time - dataset.data[i - 1].odom_time;
		calib_v = dataset.data[i].v * dataset.odom_calib.mult_v;
		calib_phi = g2o::normalize_theta(dataset.data[i].phi * dataset.odom_calib.mult_phi + dataset.odom_calib.add_phi);

		ackerman_motion_model(x, y, th, calib_v, calib_phi, dt);
        dead_reckoning.push_back(SE2(x, y, th));
	}
}


void
add_relative_edge(
    int vertex_from, int vertex_to, SE2 relative_pose,  
    Matrix3d information_matrix,
	SparseOptimizer &optimizer)
{
	EdgeSE2* edge = new EdgeSE2;
	edge->vertices()[0] = optimizer.vertex(vertex_from);
	edge->vertices()[1] = optimizer.vertex(vertex_to);
	edge->setMeasurement(relative_pose);
	edge->setInformation(information_matrix);
	optimizer.addEdge(edge);
}


void 
add_global_edge(
    int vertex, SE2 pose,  
    Matrix3d information_matrix,
	SparseOptimizer &optimizer)
{
	EdgeGPS *edge_gps = new EdgeGPS;
	edge_gps->vertices()[0] = optimizer.vertex(vertex);
	edge_gps->setMeasurement(pose);
	edge_gps->setInformation(information_matrix);
	optimizer.addEdge(edge_gps);
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


void
save_corrected_vertices(
	DatasetCarmen &data, 
	vector<int> &vertices_ids, 
	SparseOptimizer *optimizer, 
	string filename, 
	double offset_x, 
	double offset_y)
{
	string name = data._path + "/" + filename;
	FILE *f = fopen(name.c_str(), "w");

	if (f == NULL)
		exit(printf("File '%s' couldn't be opened!\n", name.c_str()));

	double x, y, th, t;

	for (size_t j = 0; j < vertices_ids.size(); j++)
	{
		VertexSE2* v = dynamic_cast<VertexSE2*>(optimizer->vertex(vertices_ids[j]));
		SE2 pose = v->estimate();

		x = pose.toVector().data()[0];
		y = pose.toVector().data()[1];
		th = pose.toVector().data()[2];
		t = data.data[j].velodyne_time;

		fprintf(f, "%ld %lf %lf %lf %lf %lf %lf %lf %lf\n", 
			j, x + offset_x, y + offset_y, th, t, 
			data.data[j].image_time, data.data[j].v,
			data.data[j].phi, data.data[j].odom_time);
	}

	fclose(f);	
}


void
run_gicp(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, 
	Matrix<double, 4, 4> *correction, 
	int *converged, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_leafed(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_leafed(new pcl::PointCloud<pcl::PointXYZRGB>);

	gicp.setMaximumIterations(200);
	gicp.setTransformationEpsilon(1e-5);
	gicp.setMaxCorrespondenceDistance(5.0);

	const double leaf_size = 0.3;
	grid.setLeafSize(leaf_size, leaf_size, leaf_size);

	grid.setInputCloud(source);
	grid.filter(*source_leafed);

	grid.setInputCloud(target);
	grid.filter(*target_leafed);

	gicp.setInputCloud(source_leafed);
	gicp.setInputTarget(target_leafed);
	gicp.align(*output);

	*correction = gicp.getFinalTransformation().cast<double>();
	*converged = gicp.hasConverged();
}

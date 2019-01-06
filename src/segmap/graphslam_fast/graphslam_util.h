
#ifndef __GRAPHSLAM_UTIL__
#define __GRAPHSLAM_UTIL__

#include <pcl/common/projection_matrix.h>
#include "g2o/types/slam2d/se2.h"
#include "g2o/core/sparse_optimizer.h"
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_util.h>
#include <Eigen/Core>

using namespace g2o;
using namespace Eigen;

// TODO: Add to a configuration file.
const double interlog_xy_std = 0.5;
const double interlog_th_std = 1e10;
const double odom_xy_std = 0.001;
const double odom_th_std = 0.0001;
const double gps_xy_std = 0.2;
const double gps_th_std = 1e10;
const double min_vel_to_add_gps = 0.5;

Matrix3d create_information_matrix(double x_std, double y_std, double z_std);
double gps_std_from_quality(int quality);
void create_dead_reckoning(DatasetCarmen &dataset, vector<SE2> &dead_reckoning);

void add_relative_edge(
    int vertex_from, int vertex_to, SE2 relative_pose,  
    Matrix3d information_matrix,
	SparseOptimizer &optimizer);

void add_global_edge(
    int vertex, SE2 pose,  
    Matrix3d information_matrix,
	SparseOptimizer &optimizer);

void add_vertex(int id, SE2 initial_guess, SparseOptimizer &optimizer);

void prepare_optimization(SparseOptimizer *optimizer);
SparseOptimizer* initialize_optimizer();
void run_optimization(SparseOptimizer* optimizer);

void
save_corrected_vertices(
	DatasetCarmen &data, 
	vector<int> &vertices_ids, 
	SparseOptimizer *optimizer, 
	string filename, 
	double offset_x, 
    double offset_y);

void
run_gicp(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, 
	Matrix<double, 4, 4> *correction, 
	int *converged, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
	double leaf_size=0.);

#endif

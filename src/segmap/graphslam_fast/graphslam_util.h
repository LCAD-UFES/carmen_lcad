#ifndef __GRAPHSLAM_UTIL__
#define __GRAPHSLAM_UTIL__

#include "g2o/types/slam2d/se2.h"
#include "g2o/core/sparse_optimizer.h"
//#include <carmen/segmap_dataset_old.h>
#include <Eigen/Core>

// TODO: Add to a configuration file.
const double interlog_xy_std = 0.5;
const double interlog_th_std = 1e10;
const double odom_xy_std = 0.001;
const double odom_th_std = 0.0001;
const double gps_xy_std = 0.2;
const double gps_th_std = 1e10;
const double min_vel_to_add_gps = 0.5;

Eigen::Matrix3d
create_information_matrix(double x_std, double y_std, double z_std);

double
gps_std_from_quality(int quality);

/*
void
create_dead_reckoning(DatasetCarmen &dataset, std::vector<g2o::SE2> &dead_reckoning);
*/

void
add_relative_edge(int vertex_from, int vertex_to, g2o::SE2 relative_pose,
									Matrix3d information_matrix, g2o::SparseOptimizer &optimizer);

void
add_global_edge(int vertex, g2o::SE2 pose, Matrix3d information_matrix,
								g2o::SparseOptimizer &optimizer);

void
add_vertex(int id, g2o::SE2 initial_guess, g2o::SparseOptimizer &optimizer);

void
prepare_optimization(g2o::SparseOptimizer *optimizer);

g2o::SparseOptimizer*
initialize_optimizer();

void
run_optimization(g2o::SparseOptimizer* optimizer);

/*
void
save_corrected_vertices(DatasetCarmen &data, std::vector<int> &vertices_ids,
												g2o::SparseOptimizer *optimizer, std::string filename,
												double offset_x, double offset_y);
*/

#endif

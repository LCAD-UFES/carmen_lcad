
#ifndef __SEGMAP_LOOP_CLOSURES_H__
#define __SEGMAP_LOOP_CLOSURES_H__

#include <opencv/cv.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "g2o/types/slam2d/se2.h"

#include <carmen/segmap_dataset.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_sensor_viewer.h>
#include <carmen/segmap_particle_filter.h>
#include <carmen/command_line.h>

class LoopRestriction
{
public:
	g2o::SE2 transform;
	int converged;
	int from;
	int to;
};


void
show_flipped_img_in_viewer(PointCloudViewer &viewer, cv::Mat &img);

void
run_viewer_if_necessary(Pose2d *pose,
												GridMap &map,
												ParticleFilter &pf,
												DataSample *sample,
												SensorPreproc &preproc,
												PointCloudViewer &viewer,
												int pf_was_updated,
												int show_particles,
												int view);


void
run_viewer_if_necessary(Pose2d *pose,
												GridMap &map,
												ParticleFilter &pf,
												pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
												PointCloudViewer &viewer,
												int pf_was_updated,
												int show_particles,
												int view,
												std::string path_to_save = "");


Eigen::Matrix<double, 4, 4>
compute_source2target_transform(Pose2d target_pose, Pose2d source_pose);


void
find_dataset_indices_for_accumulating_data(NewCarmenDataset &target_dataset,
																					 int target_id,
																					 double dist_accumulate_target_cloud,
																					 int *start, int *end);


void
create_target_accumulating_clouds(NewCarmenDataset &target_dataset,
                                  SensorPreproc &target_preproc,
                                  int target_id,
                                  double dist_accumulate_target_cloud,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target);


void
search_for_loop_closure_using_pose_dist(NewCarmenDataset &dataset,
                                        Pose2d reference_pose,
                                        double reference_pose_time,
                                        int from,
                                        int to,
                                        double max_dist_threshold,
                                        double min_time_threshold,
                                        int *nn_id,
                                        double min_velocity = 0);


void
run_icp_step(NewCarmenDataset &target_dataset,
             NewCarmenDataset &source_dataset,
             int target_id,
             int source_id,
             Eigen::Matrix<double, 4, 4> *relative_transform,
             int *convergence_flag,
             SensorPreproc &target_preproc,
             SensorPreproc &source_preproc,
             double voxel_grid_size,
             double dist_accumulate_target_cloud,
             bool view);


void
run_pf_step(NewCarmenDataset &target_dataset,
            NewCarmenDataset &source_dataset,
            int target_id,
            int source_id,
            Eigen::Matrix<double, 4, 4> *relative_transform,
            int *convergence_flag,
            SensorPreproc &target_preproc,
            SensorPreproc &source_preproc,
						ParticleFilter &pf,
						GridMap &map,
						PointCloudViewer &viewer,
            double dist_accumulate_target_cloud,
						int n_corrections_when_reinit,
            bool view);


void
save_output(std::string path,
            NewCarmenDataset &reference_dataset,
            std::vector<std::pair<int, int>> &indices,
            std::vector<Eigen::Matrix<double, 4, 4>> &relative_transform_vector,
            std::vector<int> &convergence_vector,
            int project_to_world = 0);


void
save_report_file(std::string path, std::vector<std::pair<int, int>> &loop_closure_indices,
                 std::vector<Eigen::Matrix<double, 4, 4>> &relative_transform_vector,
                 std::vector<int> &convergence_vector);


void
estimate_loop_closures_with_particle_filter_in_map(NewCarmenDataset &dataset,
																									std::string dataset_path,
																									std::vector<std::pair<int, int>> &loop_closure_indices,
																									std::vector<Eigen::Matrix<double, 4, 4>> *relative_transform_vector,
																									std::vector<int> *convergence_vector,
																									int n_corrections_when_reinit,
																									CommandLineArguments &args);

void
estimate_loop_closures_with_particle_filter_in_map_with_smart_loop_closure_detection(
		NewCarmenDataset &dataset, std::string dataset_path, std::vector<std::pair<int, int>> &loop_closure_indices,
		std::vector<Eigen::Matrix<double, 4, 4>> *relative_transform_vector, std::vector<int> *convergence_vector,
		int n_corrections_when_reinit, CommandLineArguments &args);

void
estimate_displacements_with_particle_filter(NewCarmenDataset &target_dataset,
																						NewCarmenDataset &dataset_to_adjust,
																						std::string target_dataset_path,
																						std::string dataset_to_adjust_path,
                                            std::vector<std::pair<int, int>> &loop_closure_indices,
                                            std::vector<Eigen::Matrix<double, 4, 4>> *relative_transform_vector,
                                            std::vector<int> *convergence_vector,
																						int n_corrections_when_reinit,
                                            CommandLineArguments &args);


void
estimate_displacements_with_particle_filter_in_map(NewCarmenDataset &target_dataset,
                                                   NewCarmenDataset &dataset_to_adjust,
                                                   std::string target_dataset_path,
                                                   std::string dataset_to_adjust_path,
                                                   std::vector<std::pair<int, int>> &loop_closure_indices,
                                                   std::vector<Eigen::Matrix<double, 4, 4>> *relative_transform_vector,
                                                   std::vector<int> *convergence_vector,
                                                   int n_corrections_when_reinit,
                                                   CommandLineArguments &args);


void
estimate_displacements_with_gicp(NewCarmenDataset &target_dataset,
																 NewCarmenDataset &dataset_to_adjust,
																 std::string target_dataset_path,
																 std::string dataset_to_adjust_path,
                                 std::vector<std::pair<int, int>> &loop_closure_indices,
                                 std::vector<Eigen::Matrix<double, 4, 4>> *relative_transform_vector,
                                 std::vector<int> *convergence_vector,
                                 CommandLineArguments &args);


#endif

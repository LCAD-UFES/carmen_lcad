
#include <vector>
#include <string>
#include <algorithm>
#include <Eigen/Core>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <carmen/util_io.h>
#include <carmen/segmap_pose2d.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_conversions.h>
#include <carmen/segmap_sensor_viewer.h>
#include <carmen/carmen_lidar_reader.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_constructors.h>
#include <carmen/segmap_args.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_particle_filter.h>
#include <carmen/segmap_particle_filter_viewer.h>

#include <carmen/command_line.h>
#include "gicp.h"

using namespace cv;
using namespace std;
using namespace pcl;
using namespace Eigen;


void
detect_loop_closures(NewCarmenDataset &dataset, vector<pair<int, int>> *loop_closure_indices,
                     double max_dist_threshold, double min_time_threshold, int step,
                     double skip_v_threshold)
{
	printf("Detecting loop closures.\n");

	int nn_id;

	if (step <= 0)
		step = 1;

	for (int i = 0; i < dataset.size(); i += step)
	{
		if (fabs(dataset[i]->v) < skip_v_threshold)
			continue;

		search_for_loop_closure_using_pose_dist(dataset,
		                                        dataset[i]->pose,
		                                        dataset[i]->time,
		                                        i + 1, dataset.size(),
		                                        max_dist_threshold,
		                                        min_time_threshold,
		                                        &nn_id);

		if (nn_id >= 0)
			loop_closure_indices->push_back(pair<int, int>(i, nn_id));
	}

	printf("Num loop closures found: %ld\n", loop_closure_indices->size());
}


void
update_map(DataSample *sample, GridMap *map, SensorPreproc &preproc)
{
	preproc.reinitialize(sample);

	for (int i = 0; i < preproc.size(); i++)
	{
		vector<PointXYZRGB> points = preproc.next_points_in_world();

		for (int j = 0; j < points.size(); j++)
			map->add_point(points[j]);
	}
}


void
run_pf_step(NewCarmenDataset &target_dataset,
            NewCarmenDataset &source_dataset,
            int target_id,
            int source_id,
            Matrix<double, 4, 4> *relative_transform,
            int *convergence_flag,
            SensorPreproc &target_preproc,
            SensorPreproc &source_preproc,
            double dist_accumulate_target_cloud,
            bool view)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	create_target_accumulating_clouds(target_dataset, target_preproc,
	                                  target_id, dist_accumulate_target_cloud,
	                                  cloud);

	GridMap map(string("/tmp/map_") + std::to_string(target_id), 50, 50, 0.2, GridMapTile::TYPE_VISUAL, 1);

	ParticleFilter pf(50,
	                  ParticleFilter::WEIGHT_VISUAL,
										1.0,
										1.0,
										degrees_to_radians(20),
										0.1,
										degrees_to_radians(0.5),
										0.01,
										0.01,
										degrees_to_radians(0.5),
										10,
										10,
										10);

	map.reload(0, 0);

	for (int i = 0; i < cloud->size(); i++)
		map.add_point(cloud->at(i));

	Matrix<double, 4, 4> smat = compute_source2target_transform(target_dataset[target_id]->pose, source_dataset[source_id]->pose);
	Pose2d source_as_pose = Pose2d::from_matrix(smat);
	pf.reset(source_as_pose.x, source_as_pose.y, source_as_pose.th);

	source_preproc.reinitialize(source_dataset[source_id]);
	load_as_pointcloud(source_preproc, cloud, SensorPreproc::CAR_REFERENCE);

	for (int i = 0; i < 5; i++)
	{
		pf.predict(0, 0, 0); // just to add a little noise
		pf.correct(cloud, map, source_dataset[source_id]->gps);
	}

	if (view)
	{
		imshow("pf_img", pf_view(pf, map, source_as_pose, pf.mode(), cloud, 1));
		waitKey(-1);
	}

	Pose2d estimate = pf.mode();
	(*relative_transform) = Pose2d::to_matrix(estimate);
	(*convergence_flag) = 1;
}


void
estimate_displacements_with_particle_filter(NewCarmenDataset &dataset,
                                            vector<pair<int, int>> &loop_closure_indices,
                                            vector<Matrix<double, 4, 4>> *relative_transform_vector,
                                            vector<int> *convergence_vector,
                                            CommandLineArguments &args)
{
	printf("Running displacement estimation using particle filters.\n");

	int i;
	int view;
	int n_processed_clouds = 0;
	int n = loop_closure_indices.size();

#ifdef _OPENMP
	view = 0;
#else
	view = args.get<int>("view");
#endif

	double dist_acc = args.get<double>("dist_to_accumulate");

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 5) default(none) private(i) \
		shared(dataset, convergence_vector, relative_transform_vector, \
					 loop_closure_indices, n_processed_clouds, n, view, args, dist_acc)
#endif
	for (i = 0; i < n; i++)
	{
		SensorPreproc preproc = create_sensor_preproc(args, &dataset, args.get<string>("log_path"));

		run_pf_step(dataset,
		            dataset,
		            loop_closure_indices[i].first,
		            loop_closure_indices[i].second,
		            &(relative_transform_vector->at(i)),
		            &(convergence_vector->at(i)),
		            preproc,
		            preproc,
		            dist_acc,
		            view);

#ifdef _OPENMP
#pragma omp critical
#endif
		{
			n_processed_clouds++;

			if (n_processed_clouds % 100 == 0)
				printf("%d processed clouds of %d\n", n_processed_clouds, n);
		}
	}
}


void
estimate_displacements_with_gicp(NewCarmenDataset &dataset,
                                 vector<pair<int, int>> &loop_closure_indices,
                                 vector<Matrix<double, 4, 4>> *relative_transform_vector,
                                 vector<int> *convergence_vector,
                                 CommandLineArguments &args)
{
	printf("Running ICPs.\n");

	int i;
	int view;
	int n_processed_clouds = 0;
	int n = loop_closure_indices.size();

#ifdef _OPENMP
	view = 0;
#else
	view = args.get<int>("view");
#endif

	double dist_acc = args.get<double>("dist_to_accumulate");
	double voxel_size = args.get<double>("voxel_size");

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 5) default(none) private(i) \
		shared(dataset, convergence_vector, relative_transform_vector, \
		       loop_closure_indices, n_processed_clouds, n, view, args, dist_acc, voxel_size)
#endif
	for (i = 0; i < n; i++)
	{
		SensorPreproc preproc = create_sensor_preproc(args, &dataset, args.get<string>("log_path"));

		run_icp_step(dataset,
		             dataset,
		             loop_closure_indices[i].first,
		             loop_closure_indices[i].second,
		             &(relative_transform_vector->at(i)),
		             &(convergence_vector->at(i)),
		             preproc,
		             preproc,
		             voxel_size,
		             dist_acc,
		             view);

#ifdef _OPENMP
#pragma omp critical
#endif
		{
			n_processed_clouds++;

			if (n_processed_clouds % 100 == 0)
				printf("%d processed clouds of %d\n", n_processed_clouds, n);
		}
	}
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;
	args.add_positional<std::string>("log_path", "Path of a log", 1);
	args.add<std::string>("odom_calib,o", "Odometry calibration file", "");
	args.add<std::string>("fused_odom,f", "Fused odometry file (optimized using graphslam)", "");
	add_default_sensor_preproc_args(args);
	add_default_gicp_args(args);
	args.save_config_file(default_data_dir() + "/loop_closures_config.txt");
	args.parse(argc, argv);

	NewCarmenDataset dataset(args.get<string>("log_path"),
	                         args.get<string>("odom_calib"),
	                         args.get<string>("fused_odom"),
	                         args.get<int>("gps_id"));

	vector<pair<int, int>> loop_closure_indices;
	detect_loop_closures(dataset,
	                     &loop_closure_indices,
	                     args.get<double>("loop_dist"),
	                     args.get<double>("time_dist"),
	                     args.get<int>("subsampling"),
	                     args.get<double>("v_thresh"));

	int size = dataset.size();
	vector<Matrix<double, 4, 4>> relative_transform_vector_gicp(size);
	vector<Matrix<double, 4, 4>> relative_transform_vector_pf(size);
	vector<int> convergence_vector_gicp(size);
	vector<int> convergence_vector_pf(size);

	estimate_displacements_with_gicp(dataset,
	                                 loop_closure_indices,
	                                 &relative_transform_vector_gicp,
	                                 &convergence_vector_gicp,
	                                 args);

//	estimate_displacements_with_particle_filter(dataset,
//	                                            loop_closure_indices,
//	                                            &relative_transform_vector_pf,
//	                                            &convergence_vector_pf,
//	                                            args);

	save_output(args.get<string>("output"),
	            dataset, dataset,
	            loop_closure_indices,
	            relative_transform_vector_gicp,
	            convergence_vector_gicp);

	printf("Done.");
	return 0;
}

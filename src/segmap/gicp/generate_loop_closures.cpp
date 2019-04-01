
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

#include <carmen/command_line.h>
#include "gicp.h"

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
		                                        i + 1,
		                                        max_dist_threshold,
		                                        min_time_threshold,
		                                        &nn_id);

		if (nn_id >= 0)
			loop_closure_indices->push_back(pair<int, int>(i, nn_id));
	}

	printf("Num loop closures found: %ld\n", loop_closure_indices->size());
}


void
estimate_displacements(NewCarmenDataset &dataset,
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

#ifdef _OPENMP
#pragma omp parallel for default(none) private(i) \
		shared(dataset, convergence_vector, relative_transform_vector, \
		       loop_closure_indices, n_processed_clouds, n, view, args)
#endif
	for (i = 0; i < n; i++)
	{
		SensorPreproc preproc = create_sensor_preproc(args, &dataset, args.get<string>("log_path"));

		run_icp_step(dataset[loop_closure_indices[i].first],
		             dataset[loop_closure_indices[i].second],
		             &(relative_transform_vector->at(i)),
		             &(convergence_vector->at(i)),
		             preproc,
		             preproc,
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

	string log_path = args.get<string>("log_path");

	FILE *out_file = safe_fopen(args.get<string>("output").c_str(), "w");
	FILE *report_file = safe_fopen(args.get<string>("report_file").c_str(), "w");

	NewCarmenDataset dataset(log_path,
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
	vector<Matrix<double, 4, 4>> relative_transform_vector(size);
	vector<int> convergence_vector(size);

	estimate_displacements(dataset,
	                       loop_closure_indices,
	                       &relative_transform_vector,
	                       &convergence_vector,
	                       args);

	save_output(args.get<string>("output"),
	            dataset, dataset,
	            loop_closure_indices,
	            relative_transform_vector,
	            convergence_vector);

	fclose (out_file);
	fclose (report_file);

	printf("Done.");
	return 0;
}

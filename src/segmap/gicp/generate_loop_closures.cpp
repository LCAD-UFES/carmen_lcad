
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
#include <carmen/segmap_loop_closures.h>

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


int
main(int argc, char **argv)
{
	CommandLineArguments args;
	args.add_positional<std::string>("log_path", "Path of a log", 1);
	args.add_positional<std::string>("param_file", "Path of the carmen.ini file", 1);
	args.add<string>("mode", "Technique for estimating displacement between loop closure poses [particle_fitler | gicp | localization]");
	args.add<std::string>("odom_calib,o", "Odometry calibration file", "none");
	args.add<std::string>("fused_odom,f", "Fused odometry file (optimized using graphslam)", "none");
	args.add<int>("n_corrections_when_reinit", "Number of correction steps when reinitializing particle filter", 20);
	add_default_sensor_preproc_args(args);
	add_default_gicp_args(args);
	add_default_localizer_args(args);
	add_default_mapper_args(args);
	args.save_config_file(default_data_dir() + "/loop_closures_config.txt");
	args.parse(argc, argv);

	string mode = args.get<string>("mode");
	string log_path = args.get<string>("log_path");
	NewCarmenDataset *dataset = create_dataset(log_path, args, "fused");

	vector<pair<int, int>> loop_closure_indices;

	if (mode.compare("localization") != 0)
	{
		detect_loop_closures(*dataset,
												 &loop_closure_indices,
												 args.get<double>("loop_dist"),
												 args.get<double>("time_dist"),
												 args.get<int>("subsampling"),
												 args.get<double>("v_thresh"));
	}

	int size = dataset->size();
	vector<Matrix<double, 4, 4>> relative_transform_vector(size);
	vector<int> convergence_vector(size);

	if (mode.compare("gicp") == 0)
	{
		estimate_displacements_with_gicp(*dataset,
																		 *dataset,
																		 log_path, log_path,
																		 loop_closure_indices,
																		 &relative_transform_vector,
																		 &convergence_vector,
																		 args);
	}
	else if (mode.compare("particle_filter") == 0)
	{
		estimate_displacements_with_particle_filter(*dataset,
																								*dataset,
																								log_path, log_path,
																								loop_closure_indices,
																								&relative_transform_vector,
																								&convergence_vector,
																								args.get<int>("n_corrections_when_reinit"),
																								args);
	}
	else if (mode.compare("localization") == 0)
	{
		estimate_loop_closures_with_particle_filter_in_map_with_smart_loop_closure_detection(
				*dataset,
				log_path,
				loop_closure_indices,
				&relative_transform_vector,
				&convergence_vector,
				args.get<int>("n_corrections_when_reinit"),
				args);
	}
	else
		exit(printf("Error: invalid mode '%s'.\n", mode.c_str()));

	save_output(args.get<string>("output"),
	            *dataset,
	            loop_closure_indices,
	            relative_transform_vector,
	            convergence_vector);

	printf("Done.");
	return 0;
}


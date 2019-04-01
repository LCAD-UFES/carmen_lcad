
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
write_output(FILE *report_file, vector<pair<int, int>> &loop_closure_indices,
             vector<Matrix<double, 4, 4>> &relative_transform_vector,
             vector<int> &convergence_vector)
{
	for (int i = 0; i < loop_closure_indices.size(); i++)
	{
		fprintf(
				report_file,
		    "%d %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
		    loop_closure_indices[i].first, loop_closure_indices[i].second,
		    convergence_vector[i], relative_transform_vector[i](0, 0),
		    relative_transform_vector[i](0, 1), relative_transform_vector[i](0, 2),
		    relative_transform_vector[i](0, 3), relative_transform_vector[i](1, 0),
		    relative_transform_vector[i](1, 1), relative_transform_vector[i](1, 2),
		    relative_transform_vector[i](1, 3), relative_transform_vector[i](2, 0),
		    relative_transform_vector[i](2, 1), relative_transform_vector[i](2, 2),
		    relative_transform_vector[i](2, 3), relative_transform_vector[i](3, 0),
		    relative_transform_vector[i](3, 1), relative_transform_vector[i](3, 2),
		    relative_transform_vector[i](3, 3));
	}
}


void
write_output_to_graphslam(FILE *out_file,
													NewCarmenDataset &dataset,
													vector<pair<int, int>> &indices,
													vector<Matrix<double, 4, 4>> &relative_transform_vector,
													vector<int> &convergence_vector)
{
	Matrix<double, 4, 4> source2target;
	Matrix<double, 4, 4> corrected_pose;

	for (int i = 0; i < indices.size(); i++)
	{
		Pose2d target_pose = dataset[indices[i].first]->pose;
		Pose2d source_pose = dataset[indices[i].second]->pose;

		source2target = compute_source2target_transform(target_pose, source_pose);
		corrected_pose = relative_transform_vector[i] * source2target;
		Pose2d pose = Pose2d::from_matrix(corrected_pose);

		fprintf(out_file, "%d %d %d %lf %lf %lf\n", indices[i].first, indices[i].second,
		        convergence_vector[i], pose.x, pose.y, pose.th);
	}
}


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
run_icps(NewCarmenDataset &dataset,
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


void
add_args(CommandLineArguments *args)
{
	add_default_sensor_preproc_args(*args);
	args->add_positional<string>("log_path", "Path of a log", 1);
	args->add_positional<string>("output", "Path of the output file", 1);
	args->add<string>("odom_calib,o", "Odometry calibration file", "");
	args->add<string>("fused_odom,f", "Fused odometry file (optimized using graphslam)", "");
	args->add<double>("voxel_size,x", "Size of voxels in voxel grid filter", 0.1);
	args->add<double>("loop_dist,d", "Maximum distance (in meters) to assume two poses form a loop closure", 2.0);
	args->add<double>("time_dist,t", "Minimum temporal difference (in seconds) to assume two poses form a loop closure (instead of being consecutive poses)", 60.0);
	args->add<int>("subsampling,s", "Number of data packages to skip when looking for loop closures (<= 1 for using all packages)", 0);
	args->add<string>("report_file,r", "Path to a file to save debug information", "/tmp/loop_closure_report.txt");
	args->add<int>("view,v", "Boolean to turn visualization on or off", 0);
	args->add<double>("v_thresh", "Skip data packages with absolute velocity below this theshold", 0);

	args->save_config_file("loop_closures_config.txt");
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;
	add_args(&args);
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

	run_icps(dataset,
					 loop_closure_indices,
					 &relative_transform_vector,
					 &convergence_vector,
					 args);

	write_output(report_file,
							 loop_closure_indices,
							 relative_transform_vector,
	             convergence_vector);

	write_output_to_graphslam(out_file, dataset, loop_closure_indices,
	                          relative_transform_vector, convergence_vector);

	fclose (out_file);
	fclose (report_file);

	printf("Done.");
	return 0;
}


#include <vector>
#include <string>
#include <Eigen/Core>
#include <carmen/command_line.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_args.h>
#include <carmen/segmap_constructors.h>
#include "gicp.h"

using namespace Eigen;
using namespace std;


void
estimate_displacements(NewCarmenDataset &reference_dataset,
                       NewCarmenDataset &dataset_to_adjust,
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

	std::string reference_log_path = args.get<string>("target");
	std::string log_to_adjust_path = args.get<string>("source");

#ifdef _OPENMP
#pragma omp parallel for default(none) private(i) \
		shared(reference_dataset, dataset_to_adjust, convergence_vector, relative_transform_vector, \
		       loop_closure_indices, n_processed_clouds, n, view, args, reference_log_path, log_to_adjust_path)
#endif
	for (i = 0; i < n; i++)
	{
		SensorPreproc ref_preproc = create_sensor_preproc(args, &reference_dataset, reference_log_path);
		SensorPreproc adj_preproc = create_sensor_preproc(args, &dataset_to_adjust, log_to_adjust_path);

		run_icp_step(reference_dataset,
		             dataset_to_adjust,
		             loop_closure_indices[i].first,
		             loop_closure_indices[i].second,
		             &(relative_transform_vector->at(i)),
		             &(convergence_vector->at(i)),
		             ref_preproc,
		             adj_preproc,
		             args.get<double>("voxel_size"),
		             5.0,
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
find_nearest_poses(NewCarmenDataset &reference_dataset,
                   NewCarmenDataset &dataset_to_adjust,
                   vector<pair<int, int>> *loop_closure_indices,
                   double max_dist_threshold,
                   int step,
                   double skip_v_threshold)
{
	printf("Detecting loop closures.\n");

	int nn_id;

	if (step <= 0)
		step = 1;

	for (int i = 0; i < dataset_to_adjust.size(); i += step)
	{
		if (fabs(dataset_to_adjust[i]->v) < skip_v_threshold)
			continue;

		search_for_loop_closure_using_pose_dist(reference_dataset,
		                                        dataset_to_adjust[i]->pose,
		                                        dataset_to_adjust[i]->time,
		                                        0, reference_dataset.size(),
		                                        max_dist_threshold, 0.0,
		                                        &nn_id);

		if (nn_id >= 0)
			// for compatibility issues, the index relative to the target shall be first.
			loop_closure_indices->push_back(pair<int, int>(nn_id, i));
	}

	printf("Num loop closures found: %ld\n", loop_closure_indices->size());
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;

	args.add_positional<string>("target", "Reference log", 1);
	args.add_positional<string>("source", "Log that will be adjusted to the reference log", 1);
	add_default_sensor_preproc_args(args);
	add_default_gicp_args(args);
	args.save_config_file(default_data_dir() + "/loop_between_logs_config.txt");
	args.parse(argc, argv);

	NewCarmenDataset* reference_dataset = create_dataset(args.get<string>("target"));
	NewCarmenDataset* dataset_to_adjust = create_dataset(args.get<string>("source"));

	SensorPreproc ref_preproc = create_sensor_preproc(args, reference_dataset, args.get<string>("target"));
	SensorPreproc adj_preproc = create_sensor_preproc(args, dataset_to_adjust, args.get<string>("source"));

	vector<pair<int, int>> loop_closure_indices;
	find_nearest_poses(*reference_dataset,
	                   *dataset_to_adjust,
	                   &loop_closure_indices,
	                   args.get<double>("loop_dist"),
	                   args.get<int>("subsampling"),
	                   args.get<double>("v_thresh"));

	int size = dataset_to_adjust->size();
	vector<int> convergence_vector(size);
	vector<Matrix<double, 4, 4>> relative_transform_vector(size);
	estimate_displacements(*reference_dataset,
	                       *dataset_to_adjust,
	                       loop_closure_indices,
	                       &relative_transform_vector,
	                       &convergence_vector,
	                       args);

	save_output(args.get<string>("output"),
	            *reference_dataset,
	            *dataset_to_adjust,
	            loop_closure_indices,
	            relative_transform_vector,
	            convergence_vector, 1);

	printf("Done.\n");
	return 0;
}

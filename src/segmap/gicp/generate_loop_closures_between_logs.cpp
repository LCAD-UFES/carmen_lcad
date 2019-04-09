
#include <vector>
#include <string>
#include <Eigen/Core>
#include <carmen/command_line.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_args.h>
#include <carmen/segmap_constructors.h>
#include "gicp.h"
#include <carmen/segmap_loop_closures.h>

using namespace Eigen;
using namespace std;


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
	args.add<string>("mode", "Technique for estimating displacement between loop closure poses [particle_filter | gicp]");
	args.add<int>("n_corrections_when_reinit", "Number of correction steps when reinitializing particle filter", 10);
	add_default_sensor_preproc_args(args);
	add_default_gicp_args(args);
	add_default_localizer_args(args);
	add_default_mapper_args(args);
	args.save_config_file(default_data_dir() + "/loop_between_logs_config.txt");
	args.parse(argc, argv);

	string reference_path = args.get<string>("target");
	string adj_path = args.get<string>("source");

	NewCarmenDataset* reference_dataset = create_dataset(reference_path);
	NewCarmenDataset* dataset_to_adjust = create_dataset(adj_path);

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
	string mode = args.get<string>("mode");

	if (mode.compare("gicp") == 0)
	{
		estimate_displacements_with_gicp(*reference_dataset,
																		 *dataset_to_adjust,
																		 reference_path, adj_path,
																		 loop_closure_indices,
																		 &relative_transform_vector,
																		 &convergence_vector,
																		 args);
	}
	else if (mode.compare("particle_filter") == 0)
	{
		estimate_displacements_with_particle_filter(*reference_dataset,
																								*dataset_to_adjust,
																								reference_path, adj_path,
																								loop_closure_indices,
																								&relative_transform_vector,
																								&convergence_vector,
																								args.get<int>("n_corrections_when_reinit"),
																								args);
	}
	else
		exit(printf("Error: invalid mode '%s'.\n", mode.c_str()));

	save_output(args.get<string>("output"),
	            *reference_dataset,
	            *dataset_to_adjust,
	            loop_closure_indices,
	            relative_transform_vector,
	            convergence_vector, 1);

	printf("Done.\n");
	return 0;
}


#include <vector>
#include <string>
#include <carmen/command_line.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_args.h>
#include <carmen/segmap_constructors.h>
#include "gicp.h"

using namespace std;


void
compute_displacements(NewCarmenDataset &reference_dataset,
											NewCarmenDataset &dataset_to_adjust,
											vector<pair<int, int>> &loop_closure_indices)
{

}


void
find_nearest_poses(NewCarmenDataset &reference_dataset,
									 NewCarmenDataset &dataset_to_adjust,
									 vector<pair<int, int>> *loop_closure_indices,
                   double max_dist_threshold,
									 int step)
{
	printf("Detecting loop closures.\n");

	int nn_id;

	if (step <= 0)
		step = 1;

	for (int i = 0; i < dataset_to_adjust.size(); i += step)
	{
		search_for_loop_closure_using_pose_dist(reference_dataset,
																						dataset_to_adjust[i]->pose,
																						dataset_to_adjust[i]->time,
																						0, max_dist_threshold, 0.0,
																						&nn_id);

		if (nn_id >= 0)
			loop_closure_indices->push_back(pair<int, int>(i, nn_id));
	}

	printf("Num loop closures found: %ld\n", loop_closure_indices->size());
}


void
add_args(CommandLineArguments &args)
{
	add_default_sensor_preproc_args(args);
	args.add_positional<string>("target", "Reference log");
	args.add_positional<string>("source", "Log that will be adjusted to the reference log");
	args.add_positional<string>("output", "Output file path");
	args.add<int>("gps_id", "Id of the gps to be used", 1);
	args.add<double>("voxel_size,v", "Size of voxels in voxel grid filter", 0.1);
	args.add<double>("loop_dist,d", "Maximum distance (in meters) to assume two poses form a loop closure", 2.0);
	args.add<int>("subsampling,s", "Number of data packages to skip when looking for loop closures (<= 1 for using all packages)", 0);
	args.add<string>("report_file,r", "Path to a file to save debug information", "/tmp/loop_closure_report.txt");
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;
	add_args(args);
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
										 args.get<int>("subsampling"));

	compute_displacements(*reference_dataset,
												 *dataset_to_adjust,
												 loop_closure_indices);

	printf("Done.\n");
	return 0;
}

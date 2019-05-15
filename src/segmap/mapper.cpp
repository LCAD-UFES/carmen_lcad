
#include <boost/filesystem.hpp>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_semantic_segmentation_viewer.h>
#include <carmen/segmap_preproc.h>
#include <carmen/util_math.h>
#include <carmen/command_line.h>
#include <carmen/segmap_args.h>
#include <carmen/segmap_constructors.h>

using namespace cv;
using namespace std;
using namespace pcl;


int
main(int argc, char **argv)
{
	string log_path;
	string map_path;

	CommandLineArguments args;

	add_default_slam_args(args);
	add_default_sensor_preproc_args(args);
	add_default_mapper_args(args);

	args.add<int>("view,v", "Flag to set visualization on or off", 1);
	args.add<int>("clean_map", "Flag for choosing to delete or not previous maps of the same region", 0);

	args.save_config_file(default_data_dir() + "/mapper_config.txt");
	args.parse(argc, argv);

	log_path = args.get<string>("log_path");
	map_path = args.get<string>("map_path");

	if (args.get<int>("clean_map") && boost::filesystem::exists(map_path))
		boost::filesystem::remove_all(map_path);

	GridMap map = create_grid_map(args, 1);
	NewCarmenDataset *dataset = create_dataset(log_path, args, "graphslam");
	SensorPreproc preproc = create_sensor_preproc(args, dataset, log_path);

	create_map(map, dataset, args.get<int>("step"), preproc,
						 args.get<double>("v_thresh"),
						 args.get<int>("view"), 950);

	printf("Done.\n");
	return 0;
}


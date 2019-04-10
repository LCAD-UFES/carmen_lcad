
#include <boost/filesystem.hpp>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_semantic_segmentation_viewer.h>
#include <carmen/segmap_preproc.h>
#include <carmen/util_math.h>
#include <carmen/command_line.h>
#include "libsegmap/initializations/segmap_args.h"
#include <carmen/segmap_constructors.h>

using namespace cv;
using namespace std;
using namespace pcl;


int
main(int argc, char **argv)
{
	int map_type;
	string log_path;

	CommandLineArguments args;
	add_default_slam_args(args);
	add_default_sensor_preproc_args(args);
	add_default_mapper_args(args);
	args.add<int>("view,v", "Flag to set visualization on or off", 1);
	args.save_config_file(default_data_dir() + "/mapper_config.txt");
	args.parse(argc, argv);

	log_path = args.get<string>("log_path");

	string odom_calib_path = default_odom_calib_path(log_path.c_str());
	string fused_odom_path = default_fused_odom_path(log_path.c_str());
	string graphslam_path = default_graphslam_path(log_path.c_str());
	string graphslam_to_map_path = default_graphslam_to_map_path(log_path.c_str());

	SensorPreproc::IntensityMode i_mode;
	i_mode = parse_intensity_mode(args.get<string>("intensity_mode"));

	if (i_mode == SensorPreproc::SEMANTIC)
		map_type = GridMapTile::TYPE_SEMANTIC;
	else
		map_type = GridMapTile::TYPE_VISUAL;

	if (boost::filesystem::exists(args.get<string>("map_path")))
		boost::filesystem::remove_all(args.get<string>("map_path"));

	GridMap map(args.get<string>("map_path"),
							args.get<double>("tile_size"),
							args.get<double>("tile_size"),
							args.get<double>("resolution"),
							map_type, 1);

	NewCarmenDataset *dataset = new NewCarmenDataset(log_path, odom_calib_path, graphslam_path);

	SensorPreproc preproc = create_sensor_preproc(args, dataset, log_path);

	create_map(map, dataset, args.get<int>("step"), preproc,
						 args.get<double>("v_thresh"),
						 args.get<int>("view"));

	printf("Done.\n");
	return 0;
}



#include <carmen/segmap_dataset.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_sensor_viewer.h>
#include <carmen/segmap_particle_filter_viewer.h>
#include <carmen/segmap_semantic_segmentation_viewer.h>
#include <carmen/segmap_preproc.h>
#include <carmen/util_time.h>
#include <carmen/util_math.h>
#include <carmen/command_line.h>

using namespace cv;
using namespace std;
using namespace pcl;


void
update_map(DataSample *sample, GridMap *map, SensorPreproc &preproc)
{
	preproc.reinitialize(sample);

	for (int i = 0; i < preproc.size(); i++)
	{
		vector<PointXYZRGB> points = preproc.next_points();

		for (int j = 0; j < points.size(); j++)
			map->add_point(points[j]);
	}
}


void
view(GridMap &map, DataSample *sample, Pose2d &offset, PointCloudViewer &viewer)
{
	Pose2d pose;
	pose = sample->pose;

	pose.x -= offset.x;
	pose.y -= offset.y;

	Mat map_img = map.to_image().clone();
	draw_pose(map, map_img, pose, Scalar(0, 255, 0));

	// flip vertically.
	Mat map_view;
	flip(map_img, map_view, 0);

	//viewer.clear();
	//viewer.show(colored);
	//viewer.show(img, "img", 640);
	//viewer.show(simg, "simg", 640);
	//viewer.show(simg_view, "simg_view", 640);
	viewer.show(map_view, "map", 640);
	viewer.loop();
}


void
create_map(GridMap &map, const char *log_path, NewCarmenDataset *dataset,
					int use_xsens, int step, SensorPreproc::IntensityMode i_mode)
{
	TimeCounter timer;
	DataSample *sample;
	PointCloudViewer viewer;
	vector<double> times;

	CarmenLidarLoader *vloader = new CarmenLidarLoader;
	CarmenImageLoader *iloader = new CarmenImageLoader;
	SemanticSegmentationLoader *sloader = new SemanticSegmentationLoader(log_path);
	Pose2d offset = dataset->at(0)->pose;

	SensorPreproc preproc(vloader, iloader, sloader,
												dataset->vel2cam(), dataset->vel2car(), dataset->projection_matrix(),
												dataset->xsens2car(), use_xsens, dataset->at(0)->pose, i_mode);

	for (int i = 0; i < dataset->size(); i += step)
	{
		sample = dataset->at(i);

		if (fabs(sample->v) < 1.0)
			continue;

		timer.start();

		map.reload(sample->pose.x - offset.x, sample->pose.y - offset.y);
		update_map(sample, &map, preproc);

		times.push_back(timer.ellapsed());

		if (times.size() % 50 == 0)
			printf("Avg ellapsed %ld: %lf Current: %lf\n",
						 times.size(),
						 mean(times),
						 times[times.size() - 1]);

		view(map, sample, offset, viewer);
	}
}



SensorPreproc::IntensityMode
parse_intensity_mode(string map_type)
{
	if (map_type.compare("remission") == 0)
		return SensorPreproc::INTENSITY;
	else if (map_type.compare("visual") == 0)
		return SensorPreproc::COLOR;
	else if (map_type.compare("semantic") == 0)
		return SensorPreproc::SEMANTIC;
	else
		exit(printf("Error: invalid map type '%s'.\n", map_type.c_str()));
}


int
main(int argc, char **argv)
{
	int grid_map_type;
	SensorPreproc::IntensityMode i_mode;
	string log_path, map_path;
	double resolution, tile_size;
	po::variables_map args;

	CommandLineArguments args_parser;

	args_parser.add_positional<string>("log_path", "Path of a log", 1);
	args_parser.add<double>("resolution,r", "Map resolution", 0.2);
	args_parser.add<double>("tile_size,t", "Map tiles size", 50);
	args_parser.add<string>("map_path,m", "Path to save the maps", "/tmp");
	args_parser.add<int>("use_xsens,x", "Whether or not to use pitch, and roll angles from xsens", 1);
	args_parser.add<int>("step,s", "Number of data packages to skip", 1);
	args_parser.add<string>("intensity_mode,i", "Type of grid map [remission | visual | semantic]", "remission");
	args_parser.save_config_file("data/mapper_config.txt");
	args_parser.parse(argc, argv);

	resolution = args_parser.get<double>("resolution");
	tile_size = args_parser.get<double>("tile_size");
	log_path = args_parser.get<string>("log_path");
	map_path = args_parser.get<string>("map_path");
	i_mode = parse_intensity_mode(args_parser.get<string>("intensity_mode"));

	printf("map path: %s\n", map_path.c_str());
	printf("tile size: %lf\n", tile_size);
	printf("resolution: %lf\n", resolution);

	grid_map_type = (i_mode == SensorPreproc::SEMANTIC) ? (GridMapTile::TYPE_SEMANTIC) : (GridMapTile::TYPE_VISUAL);
	GridMap map(map_path, tile_size, tile_size, resolution, grid_map_type, 1);

	string odom_calib_path = default_odom_calib_path(log_path.c_str());
	string fused_odom_path = default_fused_odom_path(log_path.c_str());
	string graphslam_path = default_graphslam_path(log_path.c_str());

	NewCarmenDataset *dataset = new NewCarmenDataset(log_path, odom_calib_path, graphslam_path);

	create_map(map, log_path.c_str(), dataset,
						 args_parser.get<int>("use_xsens"),
						 args_parser.get<int>("step"),
						 i_mode);

	printf("Done\n");
	return 0;
}



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
		vector<PointXYZRGB> points = preproc.next_points_in_world();

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
create_map(GridMap &map, NewCarmenDataset *dataset, int step,
					 SensorPreproc &preproc, Pose2d &offset, double skip_velocity_threshold)
{
	TimeCounter timer;
	DataSample *sample;
	PointCloudViewer viewer;
	vector<double> times;

	for (int i = 0; i < dataset->size(); i += step)
	{
		sample = dataset->at(i);

		if (fabs(sample->v) < skip_velocity_threshold)
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


void
add_args_for_parsing(CommandLineArguments &args)
{
	args.add_positional<string>("log_path", "Path of a log", 1);
	args.add<double>("v_thresh", "Skip data packages with absolute velocity below this theshold", 1.0);

	args.add<double>("resolution,r", "Map resolution", 0.2);
	args.add<double>("tile_size,t", "Map tiles size", 50);
	args.add<string>("map_path,m", "Path to save the maps", "/tmp");
	args.add<int>("use_xsens,x", "Whether or not to use pitch, and roll angles from xsens", 1);
	args.add<int>("step,s", "Number of data packages to skip", 1);
	args.add<string>("intensity_mode,i", "Type of grid map [remission | visual | semantic]", "remission");
	args.add<double>("ignore_above_threshold", "Ignore points with z-coord above this threshold", DBL_MAX);
	args.add<double>("ignore_below_threshold", "Ignore points with z-coord below this threshold", -DBL_MAX);
	args.add<double>("offset_x", "Offset to subtract the pose (x-coord)", 7757888.199148);
	args.add<double>("offset_y", "Offset to subtract the pose (y-coord)", -363560.975411);
	args.save_config_file("data/mapper_config.txt");
}


int
main(int argc, char **argv)
{
	int map_type;
	string log_path;

	CommandLineArguments args;
	add_args_for_parsing(args);
	args.parse(argc, argv);

	log_path = args.get<string>("log_path");

	string odom_calib_path = default_odom_calib_path(log_path.c_str());
	string fused_odom_path = default_fused_odom_path(log_path.c_str());
	string graphslam_path = default_graphslam_path(log_path.c_str());

	SensorPreproc::IntensityMode i_mode;
	i_mode = parse_intensity_mode(args.get<string>("intensity_mode"));

	if (i_mode == SensorPreproc::SEMANTIC)
		map_type = GridMapTile::TYPE_SEMANTIC;
	else
		map_type = GridMapTile::TYPE_VISUAL;

	GridMap map(args.get<string>("map_path"),
							args.get<double>("tile_size"),
							args.get<double>("tile_size"),
							args.get<double>("resolution"),
							map_type, 1);

	NewCarmenDataset *dataset = new NewCarmenDataset(log_path, odom_calib_path, graphslam_path);

	CarmenLidarLoader *vloader = new CarmenLidarLoader;
	CarmenImageLoader *iloader = new CarmenImageLoader;
	SemanticSegmentationLoader *sloader = new SemanticSegmentationLoader(log_path);
	Pose2d offset = dataset->at(0)->pose;

	SensorPreproc preproc(vloader, iloader, sloader,
												dataset->vel2cam(), dataset->vel2car(), dataset->projection_matrix(),
												dataset->xsens2car(), args.get<int>("use_xsens"), dataset->at(0)->pose, i_mode);

	create_map(map, dataset, args.get<int>("step"), preproc, offset,
						 args.get<double>("v_thresh"));

	delete(vloader);
	delete(iloader);
	delete(sloader);

	printf("Done.\n");
	return 0;
}


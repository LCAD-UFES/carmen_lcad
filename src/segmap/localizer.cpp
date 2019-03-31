
#include <string>
#include <opencv/cv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <carmen/util_math.h>
#include <carmen/command_line.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_particle_filter.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_conversions.h>
#include <carmen/segmap_particle_filter_viewer.h>
#include <carmen/segmap_sensor_viewer.h>

using namespace cv;
using namespace std;
using namespace pcl;


void
viewer(DataSample *sample, ParticleFilter &pf, GridMap &map, int step, int n_total_steps,
			 PointCloud<PointXYZRGB>::Ptr cloud, Pose2d offset,
			 PointCloudViewer &s_viewer)
{
	Mat view_img;
	Mat pf_view_img;

	Pose2d gt_pose = sample->pose;
	gt_pose.x -= offset.x;
	gt_pose.y -= offset.y;

	Pose2d mean = pf.mean();
	Pose2d mode = pf.mode();

	printf("Step: %d of %d ", step, n_total_steps);
	printf("GT_pose: %lf %lf %lf ", gt_pose.x, gt_pose.y, gt_pose.th);
	printf("PF_Mean: %lf %lf %lf ", mean.x, mean.y, mean.th);
	printf("PF_Mode: %lf %lf %lf ", mode.x, mode.y, mode.th);
	printf("D_GT_MEAN: %lf ", dist2d(mean.x, mean.y, gt_pose.x, gt_pose.y));
	printf("D_GT_MODE: %lf ", dist2d(mode.x, mode.y, gt_pose.x, gt_pose.y));
	printf("O_GT_MEAN: %lf ", fabs(normalize_theta(mean.th - gt_pose.th)));
	printf("O_GT_MODE: %lf ", fabs(normalize_theta(mode.th - gt_pose.th)));
	printf("\n");
	fflush(stdout);

	Mat pf_img = pf_view(pf, map, gt_pose, cloud, 1);

	//Mat concat;
	//hconcat(pf_view_img, view_img, concat);
	////sprintf(img_name, "%s/step_%010d.png", path_save_maps, i);
	//char text[32];
	//sprintf(text, "DistGT: %.2lfm Vel: %.2lfm/s", dist2d(mean.x, mean.y, gt_pose.x, gt_pose.y), sample->v);
	//putText(concat, text, Point(780, 700), FONT_HERSHEY_PLAIN, 1.3, Scalar(255,255,255), 1);
	////imwrite(img_name, concat);
	Mat flipped;
	flip(pf_img, flipped, 0);

	s_viewer.show(flipped, "bla");
	s_viewer.loop();
}


void
run_particle_filter(ParticleFilter &pf, GridMap &map,
										NewCarmenDataset *dataset,
										SensorPreproc &preproc,
										int step,
										double skip_velocity_threshold,
										int correction_step,
										int steps_to_skip_map_reload,
										int min_speed_for_reloading_map)
{
	double dt;
	DataSample *sample;
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloudViewer s_viewer;
	Pose2d pose;
	Pose2d p0 = dataset->at(0)->pose;

	p0.x = p0.y = 0.0;

	pf.reset(p0.x, p0.y, p0.th);
	map.reload(p0.x, p0.y);

	int n = 0;
	int do_correction;
	int last_reload = 0;

	for (int i = step; i < dataset->size(); i += step)
	{
		sample = dataset->at(i);

		if (fabs(sample->v) < skip_velocity_threshold)
			continue;

		dt = sample->image_time - dataset->at(i - step)->image_time;
		pf.predict(sample->v, sample->phi, dt);

		preproc.reinitialize(sample);
		load_as_pointcloud(preproc, cloud, SensorPreproc::CAR_REFERENCE);

		do_correction = 0;

		if (correction_step <= 1)
			do_correction = 1;
		else if (n % correction_step == 0)
			do_correction = 1;

		if (do_correction)
			pf.correct(cloud, map, sample->gps);

		pose = pf.mean();

		// only reload the map if the car is moving, and after a while.
		// this prevents frequent (expensive) reloads when near to borders.
		if ((fabs(sample->v) > min_speed_for_reloading_map)
				&& (n - last_reload > steps_to_skip_map_reload))
		{
			map.reload(pose.x, pose.y);
			last_reload = i;
		}

		viewer(sample, pf, map, i, dataset->size(), cloud,
					 dataset->at(0)->pose, s_viewer);

		n++;
	}
}


void
add_args_for_parsing(CommandLineArguments &args)
{
	args.add_positional<string>("log_path", "Path of a log", 1);
	args.add<double>("v_thresh", "Skip data packages with absolute velocity below this theshold", 0);

	// map parameters
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

	// localization parameters
	args.add<int>("n_particles", "Number of particles", 30);
	args.add<int>("use_gps_weight", "Flag to choose if GPS data should be used for weighting particles", 0);
	args.add<int>("use_map_weight", "Flag to choose if the map should be used for weighting particles", 1);
	args.add<double>("gps_xy_std", "Std of gps position (m)", 0.5);
	args.add<double>("gps_h_std", "Std of gps heading estimates (degrees)", 5);
	args.add<double>("v_std", "Std of linear velocity measurements (m/s)", 0.2);
	args.add<double>("phi_std", "Std of steering wheel angle measurements (degrees)", 0.5);
	args.add<double>("odom_xy_std", "Std of dead reckoning position estimates (m)", 0.1);
	args.add<double>("odom_h_std", "Std of dead reckoning heading estimates (degrees)", 0.5);
	args.add<double>("color_red_std", "Std of pixel color measurements", 10.);
	args.add<double>("color_green_std", "Std of pixel color measurements", 10.);
	args.add<double>("color_blue_std", "Std of pixel color measurements", 10.);
	args.add<int>("seed", "Seed for pseudo-random number generator", 0);
	args.add<int>("correction_step", "Frequency in which correction takes place [<= 1 for always correcting]", 1);
	args.add<double>("min_speed_for_reloading_map", "Do not perform map reloading from disk if velocity is smaller than this threshold", 0.5);
	args.add<int>("steps_to_skip_map_reload", "Minimum number of steps to wait until a new map reload from disk", 10);

	args.save_config_file("data/localizer_config.txt");
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


GridMap
create_grid_map(CommandLineArguments &args)
{
	int map_type;
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
							map_type, 0);


	return map;
}


ParticleFilter
create_particle_filter(CommandLineArguments &args)
{
	ParticleFilter::WeightType wtype;

	if (args.get<int>("use_gps_weight"))
		wtype = ParticleFilter::WEIGHT_GPS;
	else if (args.get<string>("intensity_mode").compare("semantic") == 0)
		wtype = ParticleFilter::WEIGHT_SEMANTIC;
	else
		wtype = ParticleFilter::WEIGHT_VISUAL;

	ParticleFilter pf(args.get<int>("n_particles"),
										wtype,
										args.get<double>("gps_xy_std"),
										args.get<double>("gps_xy_std"),
										degrees_to_radians(args.get<double>("gps_h_std")),
										args.get<double>("v_std"),
										degrees_to_radians(args.get<double>("phi_std")),
										args.get<double>("odom_xy_std"),
										args.get<double>("odom_xy_std"),
										degrees_to_radians(args.get<double>("odom_h_std")),
										args.get<double>("color_red_std"),
										args.get<double>("color_green_std"),
										args.get<double>("color_blue_std"));

	return pf;
}


NewCarmenDataset*
create_dataset(CommandLineArguments &args)
{
	string log_path = args.get<string>("log_path");
	string odom_calib_path = default_odom_calib_path(log_path.c_str());
	string fused_odom_path = default_fused_odom_path(log_path.c_str());
	string graphslam_path = default_graphslam_path(log_path.c_str());

	NewCarmenDataset *dataset = new NewCarmenDataset(log_path, odom_calib_path, graphslam_path);

	return dataset;
}


SensorPreproc
create_sensor_preproc(CommandLineArguments &args, NewCarmenDataset *dataset)
{
	CarmenLidarLoader *vloader = new CarmenLidarLoader;
	CarmenImageLoader *iloader = new CarmenImageLoader;
	SemanticSegmentationLoader *sloader = new SemanticSegmentationLoader(args.get<string>("log_path"));

	SensorPreproc::IntensityMode i_mode;
	i_mode = parse_intensity_mode(args.get<string>("intensity_mode"));

	SensorPreproc preproc(vloader, iloader, sloader,
												dataset->vel2cam(), dataset->vel2car(), dataset->projection_matrix(),
												dataset->xsens2car(), args.get<int>("use_xsens"), dataset->at(0)->pose,
												i_mode);


	return preproc;
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;
	add_args_for_parsing(args);
	args.parse(argc, argv);

	NewCarmenDataset* dataset = create_dataset(args);
	SensorPreproc preproc = create_sensor_preproc(args, dataset);

	GridMap map = create_grid_map(args);
	ParticleFilter pf = create_particle_filter(args);

	pf.seed(args.get<int>("seed"));
	run_particle_filter(pf, map, dataset, preproc,
											args.get<int>("step"),
											args.get<double>("v_thresh"),
											args.get<int>("correction_step"),
											args.get<double>("min_speed_for_reloading_map"),
											args.get<int>("steps_to_skip_map_reload"));

	printf("Done.\n");
	return 0;
}


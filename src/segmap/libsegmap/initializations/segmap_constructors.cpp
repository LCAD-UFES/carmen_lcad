
#include <string>
#include <carmen/command_line.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_particle_filter.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_conversions.h>
#include <carmen/segmap_constructors.h>
#include <carmen/segmap_args.h>

using namespace std;


string
poses_path_from_pose_mode(string pose_mode, string log_path)
{
	string path;

	if (pose_mode.compare("fused") == 0)
		path = default_fused_odom_path(log_path.c_str());
	else if (pose_mode.compare("graphslam") == 0)
		path = default_graphslam_path(log_path.c_str());
	else if (pose_mode.compare("graphslam_to_map") == 0)
		path = default_graphslam_to_map_path(log_path.c_str());
	else
		exit(printf("Error: Invalid mode '%s'\n.", pose_mode.c_str()));

	return path;
}


GridMap
create_grid_map(CommandLineArguments &args, int save_map)
{
	SensorPreproc::IntensityMode i_mode;
	GridMapTile::MapType m_type;

	i_mode = parse_intensity_mode(args.get<string>("intensity_mode"));
	m_type = map_type_from_intensity_mode(i_mode);

	GridMap map(args.get<string>("map_path"),
							args.get<double>("tile_size"),
							args.get<double>("tile_size"),
							args.get<double>("resolution"),
							m_type, save_map);

	return map;
}


ParticleFilter
create_particle_filter(CommandLineArguments &args)
{
	ParticleFilter::WeightType w_type;
	w_type = parse_weight_type(args);

	ParticleFilter pf(args.get<int>("n_particles"), w_type,
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
create_dataset(string log_path, double camera_latency, string mode)
{
	string poses_path, odom_calib_path;

	poses_path = poses_path_from_pose_mode(mode, log_path);
	odom_calib_path = default_odom_calib_path(log_path.c_str());

	NewCarmenDataset *dataset = new NewCarmenDataset(log_path, odom_calib_path, poses_path, camera_latency);

	return dataset;
}


SensorPreproc
create_sensor_preproc(CommandLineArguments &args,
											NewCarmenDataset *dataset,
											string log_path)
{
	string icalib_path;

	if (args.get<int>("use_calib"))
		icalib_path = default_intensity_calib_path(log_path.c_str());
	else
		icalib_path = "none";

	CarmenLidarLoader *vloader = new CarmenLidarLoader();
	CarmenImageLoader *iloader = new CarmenImageLoader;
	SemanticSegmentationLoader *sloader = new SemanticSegmentationLoader(log_path);

	SensorPreproc::IntensityMode i_mode;
	i_mode = parse_intensity_mode(args.get<string>("intensity_mode"));

	double above = args.get<double>("ignore_above_threshold");
	double below = args.get<double>("ignore_below_threshold");

	if (i_mode == SensorPreproc::SEMANTIC)
	{
		above = DBL_MAX;
		below = -DBL_MAX;
	}

	SensorPreproc preproc(vloader, iloader, sloader,
												dataset->vel2cam(), dataset->vel2car(), dataset->projection_matrix(),
												dataset->xsens2car(), args.get<int>("use_xsens"),
												i_mode,
												icalib_path,
												above,
												below);

	preproc.set_lane_mark_detection(args.get<int>("segment_lane_marks"));

	return preproc;
}




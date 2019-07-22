
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
	GridMapTile::MapType map_type = parse_map_type(args.get<string>("map_type"));

	GridMap map(args.get<string>("map_path"),
							args.get<double>("tile_size"),
							args.get<double>("tile_size"),
							args.get<double>("resolution"),
							map_type, save_map);

	return map;
}


ParticleFilter
create_particle_filter(CommandLineArguments &args)
{
	ParticleFilter pf(args.get<int>("n_particles"), 
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
					args.get<double>("color_blue_std"),
					args.get<double>("reflectivity_std"));
	
	pf.set_use_gps_weight(args.get<int>("use_gps_weight"));
	pf.set_use_ecc_weight(args.get<int>("use_ecc_weight"));
	pf.set_use_map_weight(args.get<int>("use_map_weight"));

	return pf;
}


NewCarmenDataset*
create_dataset(string log_path, CommandLineArguments &args, string pose_mode)
{
	double camera_latency = args.get<double>("camera_latency");
	string intensity_mode = args.get<string>("intensity_mode");

	string poses_path, odom_calib_path;

	poses_path = poses_path_from_pose_mode(pose_mode, log_path);
	odom_calib_path = default_odom_calib_path(log_path.c_str());

	NewCarmenDataset::SyncSensor sync_sensor;
	SensorPreproc::IntensityMode i_mode;
	i_mode = parse_intensity_mode(intensity_mode);

	if (i_mode == SensorPreproc::COLOUR || i_mode == SensorPreproc::SEMANTIC)
		sync_sensor = NewCarmenDataset::SYNC_BY_CAMERA;
	else
		sync_sensor = NewCarmenDataset::SYNC_BY_VELODYNE;

	int gps_id = args.get<int>("gps_id");

	NewCarmenDataset *dataset = new NewCarmenDataset(
			log_path,
			args.get<string>("param_file"),
			odom_calib_path,
			poses_path, camera_latency,
			gps_id,
			sync_sensor);

	return dataset;
}


SensorPreproc
create_sensor_preproc(CommandLineArguments &args,
											NewCarmenDataset *dataset,
											string log_path)
{
	string icalib_path;

	if (args.get<int>("use_calib"))
		icalib_path = default_intensity_calib_path();
	else
		icalib_path = "none";

	CarmenLidarLoader *vloader = new CarmenLidarLoader();
	CarmenImageLoader *iloader = new CarmenImageLoader;
	SemanticSegmentationLoader *sloader = new SemanticSegmentationLoader(log_path);

	SensorPreproc::IntensityMode i_mode;
	i_mode = parse_intensity_mode(args.get<string>("intensity_mode"));

	double above = args.get<double>("ignore_above_threshold");
	double below = args.get<double>("ignore_below_threshold");

//	if (i_mode == SensorPreproc::SEMANTIC || i_mode == SensorPreproc::COLOUR)
//	{
//		above = DBL_MAX;
//		below = -DBL_MAX;
//	}

	SensorPreproc preproc(vloader, iloader, sloader,
												dataset->vel2cam(), dataset->vel2car(), dataset->projection_matrix(),
												dataset->xsens2car(), args.get<int>("use_xsens"),
												i_mode,
												icalib_path,
												above,
												below);

	preproc.set_lane_mark_detection(args.get<int>("segment_lane_marks"));
	preproc.set_semantic_remapping_flag(args.get<int>("use_semantic_remapping"));

	return preproc;
}




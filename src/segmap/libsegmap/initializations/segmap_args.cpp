
#include <cstdlib>
#include <string>
#include <carmen/segmap_args.h>
#include <carmen/command_line.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_particle_filter.h>

using namespace std;


SensorPreproc::IntensityMode
parse_intensity_mode(string map_type)
{
	if (map_type.compare("remission") == 0)
		return SensorPreproc::INTENSITY;
	else if (map_type.compare("visual") == 0)
		return SensorPreproc::COLOR;
	else if (map_type.compare("semantic") == 0)
		return SensorPreproc::SEMANTIC;
	else if (map_type.compare("raw") == 0)
		return SensorPreproc::RAW_INTENSITY;
	else if (map_type.compare("bright") == 0)
		return SensorPreproc::BRIGHT;
	else
		exit(printf("Error: invalid map type '%s'.\n", map_type.c_str()));
}


GridMapTile::MapType
map_type_from_intensity_mode(SensorPreproc::IntensityMode i_mode)
{
	if (i_mode == SensorPreproc::SEMANTIC)
		return GridMapTile::TYPE_SEMANTIC;
	else
		return GridMapTile::TYPE_VISUAL;
}


ParticleFilter::WeightType
parse_weight_type(CommandLineArguments &args)
{
	ParticleFilter::WeightType w_type;

	if (args.get<int>("use_gps_weight"))
		w_type = ParticleFilter::WEIGHT_GPS;
	else if (args.get<string>("intensity_mode").compare("semantic") == 0)
		w_type = ParticleFilter::WEIGHT_SEMANTIC;
	else
		w_type = ParticleFilter::WEIGHT_VISUAL;

	return w_type;
}


string
default_data_dir()
{
	return string(getenv("CARMEN_HOME")) + "/src/segmap/data/";
}


void
add_default_slam_args(CommandLineArguments &args)
{
	args.add_positional<string>("log_path", "Path of a log", 1);
	args.add<double>("v_thresh", "Skip data packages with absolute velocity below this theshold", 0);
	args.add<int>("step,s", "Number of data packages to skip", 1);
}


void
add_default_mapper_args(CommandLineArguments &args)
{
	// map parameters
	args.add<double>("resolution,r", "Map resolution", 0.2);
	args.add<double>("tile_size,t", "Map tiles size", 50);
	args.add<string>("map_path,m", "Path to save the maps", "/tmp/map");

	args.save_config_file(default_data_dir() + "/mapper_config.txt");
}


void
add_default_localizer_args(CommandLineArguments &args)
{
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
	args.add<int>("steps_to_skip_map_reload", "Minimum number of steps to wait until a new map reload from disk", 5);

	args.save_config_file(default_data_dir() + "/localizer_config.txt");
}


void
add_default_sensor_preproc_args(CommandLineArguments &args)
{
	args.add<double>("ignore_above_threshold", "Ignore points with z-coord (in sensor reference) above this threshold", -0.3);
	args.add<double>("ignore_below_threshold", "Ignore points with z-coord (in sensor reference) below this threshold", -DBL_MAX);
	args.add<double>("offset_x", "Offset to subtract the pose (x-coord)", 7757888.199148);
	args.add<double>("offset_y", "Offset to subtract the pose (y-coord)", -363560.975411);
	args.add<int>("use_xsens,x", "Whether or not to use pitch, and roll angles from xsens", 1);
	args.add<int>("gps_id", "Id of the gps to be used", 1);
	args.add<string>("intensity_mode,i", "What type of information to assign to LiDAR rays intensity [remission | visual | semantic | raw | bright]", "raw");
	args.add<int>("use_calib", "Flag to choose using the velodyne calibration or not.", 1);
	args.add<int>("segment_lane_marks", "Flag for choosing to segment lane marks with threshold image processing.", 0);
	args.add<double>("camera_latency", "Camera latency in seconds", 0.0);

	args.save_config_file(default_data_dir() + "/preproc_config.txt");
}


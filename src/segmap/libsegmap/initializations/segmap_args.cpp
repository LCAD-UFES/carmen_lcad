
#include <cstdlib>
#include <string>
#include <carmen/segmap_args.h>
#include <carmen/command_line.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_particle_filter.h>

using namespace std;


SensorPreproc::IntensityMode
parse_intensity_mode(string intensity_mode)
{
	if (intensity_mode.compare("reflectivity") == 0)
		return SensorPreproc::REFLECTIVITY;
	else if (intensity_mode.compare("colour") == 0)
		return SensorPreproc::COLOUR;
	else if (intensity_mode.compare("semantic") == 0)
		return SensorPreproc::SEMANTIC;
	else
		exit(printf("Invalid intensity mode '%s'\n", intensity_mode.c_str()));
}


GridMapTile::MapType
parse_map_type(string map_type)
{
	if (map_type.compare("reflectivity") == 0)
		return GridMapTile::TYPE_REFLECTIVITY;
	else if (map_type.compare("colour") == 0)
		return GridMapTile::TYPE_VISUAL;
	else if (map_type.compare("semantic") == 0)
		return GridMapTile::TYPE_SEMANTIC;
	else if (map_type.compare("occupancy") == 0)
		return GridMapTile::TYPE_OCCUPANCY;
	else
		exit(printf("Invalid map type '%s'\n", map_type.c_str()));
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
	args.add_positional<string>("param_file", "Path to the carmen.ini file", 1);
	args.add<double>("v_thresh", "Skip data packages with absolute velocity below this theshold", 0);
	args.add<int>("step,s", "Number of data packages to skip", 1);
}


void
add_default_mapper_args(CommandLineArguments &args)
{
	// map parameters
	args.add<double>("resolution,r", "Map resolution", 0.2);
	args.add<double>("tile_size,t", "Map tiles size", 70);
	args.add<int>("save_maps", "Flag for choosing to save or not the maps", 1);
	args.add<int>("clean_map", "Flag for choosing to delete or not previous maps of the same region", 0);

	args.add<int>("view,v", "Flag to set visualization on or off", 1);
	args.add<int>("viewer_width", "Width of the map's visualization", 600);
	args.add<int>("view_pointcloud", "Visualization flag.", 0);
	args.add<int>("view_imgs", "Visualization flag.", 0);
	args.add<int>("start_paused", "Flag for choosing to pause the viewer or not in the beginning", 0);

	args.add<int>("build_occupancy_map", "Flag for choosing to build or not this type of map.", 1);
	args.add<int>("build_semantic_map", "Flag for choosing to build or not this type of map.", 1);
	args.add<int>("build_visual_map", "Flag for choosing to build or not this type of map.", 1);
	args.add<int>("build_reflectivity_map", "Flag for choosing to build or not this type of map.", 1);

	//args.add<string>("map_path,m", "Path to save the maps", "/tmp/map");

	/* 
	// The following approach seems more correct...
	args.add<int>("create_reflectivity_map", "Wheter or not to create reflectivity maps.", 1);
	args.add<int>("create_occupancy_map", "Wheter or not to create occupancy maps.", 0);
	args.add<int>("create_colour_map", "Wheter or not to create colour maps.", 0);
	args.add<int>("create_semantic_map", "Wheter or not to create semantic maps.", 0);
	args.add<string>("map_type", "[semantic, colour, occupancy, reflectivity]", "reflectivity");
	*/

}


void
add_default_localizer_args(CommandLineArguments &args)
{
	// localization parameters
	args.add<int>("n_particles", "Number of particles", 50);

	args.add<int>("use_gps_weight", "Flag to choose if GPS data should be used for weighting particles", 0);
	args.add<int>("use_map_weight", "Flag to choose if the map should be used for weighting particles", 1);
	args.add<int>("use_ecc_weight", "Flag to choose if the ECC should be used for weighting particles", 0);

	args.add<double>("gps_xy_std", "Std of gps position (m)", 0.5);
	args.add<double>("gps_h_std", "Std of gps heading estimates (degrees)", 5);

	args.add<double>("v_std", "Std of linear velocity measurements (m/s)", 0.2);
	args.add<double>("phi_std", "Std of steering wheel angle measurements (degrees)", 0.5);
	
	args.add<double>("odom_xy_std", "Std of dead reckoning position estimates (m)", 0.1);
	args.add<double>("odom_h_std", "Std of dead reckoning heading estimates (degrees)", 0.5);
	
	args.add<double>("color_red_std", "Std of pixel color measurements", 3.);
	args.add<double>("color_green_std", "Std of pixel color measurements", 3.);
	args.add<double>("color_blue_std", "Std of pixel color measurements", 3.);

	args.add<double>("reflectivity_std", "Std of pixel color measurements", 3.);

	args.add<int>("seed", "Seed for pseudo-random number generator", 0);
	args.add<int>("correction_step", "Frequency in which correction takes place [<= 1 for always correcting]", 1);
	args.add<int>("steps_to_skip_map_reload", "Minimum number of steps to wait until a new map reload from disk", 5);
	
	args.add<string>("save_dir", "Directory for saving images of the localization execution", "");

}


void
add_default_sensor_preproc_args(CommandLineArguments &args)
{
	args.add<double>("ignore_above_threshold", "Ignore points with z-coord (in sensor reference) above this threshold", -0.3);
	args.add<double>("ignore_below_threshold", "Ignore points with z-coord (in sensor reference) below this threshold", -DBL_MAX);
	args.add<double>("offset_x", "Offset to subtract the pose (x-coord)", 0); //7757888.199148);
	args.add<double>("offset_y", "Offset to subtract the pose (y-coord)", 0); //-363560.975411);
	args.add<int>("use_xsens,x", "Whether or not to use pitch, and roll angles from xsens", 1);
	args.add<int>("gps_id", "Id of the gps to be used", 1);
	args.add<string>("intensity_mode,i", "intensity mode [reflectivity, colour, semantic]", "reflectivity");
	args.add<int>("use_calib", "Flag to choose using the velodyne calibration or not.", 1);
	args.add<int>("segment_lane_marks", "Flag for choosing to segment lane marks with threshold image processing.", 0);
	args.add<double>("camera_latency", "Camera latency in seconds", 0.0);
	args.add<int>("use_semantic_remapping", "Flag to use semantic remapping or not.", 1);

}


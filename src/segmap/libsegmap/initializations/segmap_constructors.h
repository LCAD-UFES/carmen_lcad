
#ifndef __SEGMAP_CONSTRUCTORS_H__
#define __SEGMAP_CONSTRUCTORS_H__

#include <string>
#include <carmen/command_line.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_particle_filter.h>
#include <carmen/segmap_dataset.h>

SensorPreproc::IntensityMode parse_intensity_mode(std::string map_type);
GridMap create_grid_map(CommandLineArguments &args, int save_map);
ParticleFilter create_particle_filter(CommandLineArguments &args);
NewCarmenDataset* create_dataset(std::string log_path, std::string mode = "graphslam");
SensorPreproc create_sensor_preproc(CommandLineArguments &args,
																		NewCarmenDataset *dataset,
																		std::string log_path);

#endif

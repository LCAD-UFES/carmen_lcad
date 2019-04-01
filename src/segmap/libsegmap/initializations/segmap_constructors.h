
#ifndef __SEGMAP_CONSTRUCTORS_H__
#define __SEGMAP_CONSTRUCTORS_H__

#include <string>
#include <carmen/command_line.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_particle_filter.h>
#include <carmen/segmap_dataset.h>

SensorPreproc::IntensityMode parse_intensity_mode(std::string map_type);
GridMap create_grid_map(CommandLineArguments &args);
ParticleFilter create_particle_filter(CommandLineArguments &args);
NewCarmenDataset* create_dataset(CommandLineArguments &args);
SensorPreproc create_sensor_preproc(CommandLineArguments &args, NewCarmenDataset *dataset);

#endif

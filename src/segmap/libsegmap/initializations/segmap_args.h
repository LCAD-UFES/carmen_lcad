
#ifndef __SEGMAP_MAPPER_ARGS_H__
#define __SEGMAP_MAPPER_ARGS_H__

#include <string>
#include <carmen/command_line.h>

std::string default_data_dir();

void add_default_slam_args(CommandLineArguments &args);
void add_default_mapper_args(CommandLineArguments &args);
void add_default_localizer_args(CommandLineArguments &args);
void add_default_sensor_preproc_args(CommandLineArguments &args);

#endif

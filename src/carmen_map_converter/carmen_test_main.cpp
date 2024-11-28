#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <carmen/grid_mapping.h>
#include <carmen/mapper_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/global_graphics.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/stereo_velodyne_interface.h>
#include <omp.h>

#include <iostream>
#include <fstream>

#include <carmen/mapper.h>
#include <carmen/carmen_stdio.h>

#define	UPDATE_CELLS_CROSSED_BY_RAYS		1
#define	DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS	0

#define HUGE_DISTANCE     32000

#define MAX_VIRTUAL_LASER_SAMPLES 10000

carmen_map_config_t map_config;
carmen_robot_ackerman_config_t car_config;
carmen_map_set_t* map_test;

bool use_merge_between_maps = false;

int main(int argc, char** argv)
{
    carmen_mapper_read_parameters(argc, argv, &map_config, &car_config);
    map_test = mapper_initialize(&map_config, car_config, use_merge_between_maps);

    return 0;
}
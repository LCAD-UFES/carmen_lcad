#ifndef LIBRANGENET_H
#define LIBRANGENET_H

// c++ stuff
#include <vector>
#include <prob_map.h>
#include <carmen/velodyne_camera_calibration.h>

void
librangenet_initialize();

int *
librangenet_process_moving_obstacles_cells(int sensor_number, carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *sensors_params);

#endif

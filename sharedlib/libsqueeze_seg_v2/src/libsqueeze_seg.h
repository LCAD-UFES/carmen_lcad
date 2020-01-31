#ifndef LIBSQUEEZESEG_H
#define LIBSQUEEZESEG_H

#include <prob_map.h>
#include <carmen/velodyne_camera_calibration.h>

void
initialize_python_context();

long long int*
libsqueeze_seg_process_point_cloud(int vertical_resolution, int shots_to_squeeze, double* point_cloud, double timestamp);

long long int *
libsqueeze_seg_process_moving_obstacles_cells(int sensor_number, carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *sensors_params);

#endif

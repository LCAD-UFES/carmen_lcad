#ifndef LIBSQUEEZESEG_H
#define LIBSQUEEZESEG_H

#include <prob_map.h>
#include <carmen/velodyne_camera_calibration.h>
#include <vector>
#include <carmen/carmen_darknet_interface.hpp>

void
initialize_python_context();

long long int*
libsqueeze_seg_process_point_cloud(int vertical_resolution, int shots_to_squeeze, double* point_cloud, double timestamp);

long long int *
libsqueeze_seg_process_moving_obstacles_cells(int sensor_number, carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *sensors_params);

void
initialize_python_dataset();

void 
libsqueeze_seg_save_txt_for_train(int vertical_resolution, int shots_to_squeeze, double* point_cloud, double timestamp);

void
libsqueeze_seg_save_npy_for_train(int vertical_resolution, int shots_to_squeeze, double* point_cloud, double timestamp);

void 
libsqueeze_seg_fill_label(int line, double label, double* data_train);

void
libsqueeze_seg_using_detections(vector<bbox_t> &predictions, vector<vector<image_cartesian>> &clustered_points, double* data_train, int vertical_resolution, int number_of_laser_shots);

double *
libsqueeze_seg_data_for_train(int sensor_number, carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *sensors_params);


#endif

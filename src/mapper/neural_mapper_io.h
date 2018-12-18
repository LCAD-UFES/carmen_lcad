/*
 * neural_mapper_io.h
 *
 *  Created on: Dec 17, 2018
 *      Author: vinicius
 */

#ifndef MAPPER_NEURAL_MAPPER_IO_H_
#define MAPPER_NEURAL_MAPPER_IO_H_

#include <iostream>
#include <fstream>
#include <math.h>
#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <carmen/grid_mapping.h>
#include "neural_map.h"
//#include "lib_neural_mapper_py.h"


void neural_mapper_initialize(int max_distance_meters, int num_clouds, carmen_map_config_t map_config);
void neural_mapper_export_dataset_as_png(bool get_next_map, char path[]);
void neural_mapper_update_queue_and_clear_maps();
bool neural_mapper_compute_travelled_distance(carmen_position_t *neural_mapper_car_position_according_to_map, carmen_pose_3D_t neural_mapper_robot_pose,
											 double x_origin, double y_origin, int neural_mapper_data_pace);
int neural_mapper_update_output_map(carmen_map_t offline_map, carmen_position_t car_pose);
void neural_mapper_update_inputs_maps_with_new_point(int x_index, int y_index, double z_meters);
int neural_mapper_update_input_maps(sensor_data_t * sensor_data, sensor_parameters_t *sensor_params, int thread_id,
									carmen_map_t *log_odds_snapshot_map, carmen_map_config_t map_config, double x_origin, double y_origin);


#endif /* MAPPER_NEURAL_MAPPER_IO_H_ */

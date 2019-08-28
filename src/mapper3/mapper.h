#ifndef MAPPER_H_
#define MAPPER_H_

#include <tf.h>
#include <carmen/moving_objects_interface.h>
#include "neural_mapper_io.h"


//#define	MINIMUM_VELODYNE_DISTANCE 0.75
#define	MINIMUM_HEIGHT_DIFFERENCE 	0.07
//#define STEREO_MAPPING_SENSOR_INDEX 	10

#define NUM_VELODYNE_POINT_CLOUDS	5
#define GLOBAL_POS_QUEUE_SIZE		100

/**
  * Prototypes
  */

void mapper_merge_online_map_with_offline_map(carmen_map_t *offline_map);
int mapper_velodyne_variable_scan(int sensor_number, carmen_velodyne_variable_scan_message *message);
int mapper_velodyne_partial_scan(int sensor_number, carmen_velodyne_partial_scan_message *velodyne_message);
void mapper_publish_map(double timestamp);
void mapper_set_robot_pose_into_the_map(carmen_localize_ackerman_globalpos_message *globalpos_message, int UPDATE_CELLS_BELOW_CAR);
void mapper_update_grid_map(carmen_point_t xt, double *zt, sensor_parameters_t *sensor_params);
void mapper_save_current_map();
void mapper_periodically_save_current_map(double timestamp);
void mapper_initialize(carmen_map_config_t *map_config, carmen_robot_ackerman_config_t main_car_config, bool use_remission_threshold);
void mapper_change_map_origin_to_another_map_block(carmen_position_t *map_origin, bool save_map);
void mapper_change_map_origin_to_another_map_block_with_clones(carmen_position_t *map_origin, bool save_map);
int run_mapper(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global);
int run_mapper(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global, double remission_threshold);
void map_decay_to_offline_map(carmen_map_t *current_map);
carmen_map_t *get_the_map();

void set_remission_threshold();
double get_remission_threshold();

carmen_map_t* mapper_get_offline_map();


#endif /* MAPPER_H_ */

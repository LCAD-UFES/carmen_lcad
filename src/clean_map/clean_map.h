#ifndef CLEAN_MAP_H_
#define CLEAN_MAP_H_

#include <tf.h>
#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <prob_interface.h>
#include <prob_measurement_model.h>

//#define	MINIMUM_VELODYNE_DISTANCE 0.75
#define	MINIMUM_HEIGHT_DIFFERENCE 	0.07
#define STEREO_MAPPING_SENSOR_INDEX 	10

#define NUM_VELODYNE_POINT_CLOUDS	5
#define GLOBAL_POS_QUEUE_SIZE		100

/**
  * Prototypes
  */

void clean_map_merge_online_map_with_offline_map(carmen_map_t *offline_map);
int clean_map_velodyne_variable_scan(int sensor_number, carmen_velodyne_variable_scan_message *message);
int clean_map_velodyne_partial_scan(carmen_velodyne_partial_scan_message *velodyne_message);
void clean_map_publish_map(double timestamp);
void clean_map_set_robot_pose_into_the_map(carmen_localize_ackerman_globalpos_message *globalpos_message, int UPDATE_CELLS_BELOW_CAR);
void clean_map_update_grid_map(carmen_point_t xt, double *zt, sensor_parameters_t *sensor_params);
void clean_map_save_current_map();
void clean_map_initialize(carmen_map_config_t *map_config, carmen_robot_ackerman_config_t main_car_config);
void clean_map_change_map_origin_to_another_map_block(carmen_position_t *map_origin);
int run_clean_map(/*sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global*/);


#endif /* clean_map_H_ */

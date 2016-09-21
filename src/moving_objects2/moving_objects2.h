#ifndef MOVING_OBJECTS2_H_
#define MOVING_OBJECTS2_H_

#include <tf.h>

//#define	MINIMUM_VELODYNE_DISTANCE 0.75
#define	MINIMUM_HEIGHT_DIFFERENCE 	0.07
#define STEREO_MAPPING_SENSOR_INDEX 	10

#define NUM_VELODYNE_POINT_CLOUDS	5
#define GLOBAL_POS_QUEUE_SIZE		100

/**
  * Prototypes
  */

void moving_objects2_merge_online_map_with_offline_map(carmen_map_t *offline_map);
int moving_objects2_velodyne_variable_scan(int sensor_number, carmen_velodyne_variable_scan_message *message);
int moving_objects2_velodyne_partial_scan(carmen_velodyne_partial_scan_message *velodyne_message);
void moving_objects2_publish_map(double timestamp);
void moving_objects2_set_robot_pose_into_the_map(carmen_localize_ackerman_globalpos_message *globalpos_message, int UPDATE_CELLS_BELOW_CAR);
void moving_objects2_update_grid_map(carmen_point_t xt, double *zt, sensor_parameters_t *sensor_params);
void moving_objects2_save_current_map();
void moving_objects2_initialize(carmen_map_config_t *map_config, carmen_robot_ackerman_config_t main_car_config);
void moving_objects2_change_map_origin_to_another_map_block(carmen_position_t *map_origin);
int run_moving_objects2(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global);


#endif /* MOVING_OBJECTS2_H_ */

#ifndef LOCALIZE_USING_MAP_H_
#define LOCALIZE_USING_MAP_H_

#include <tf.h>

//#define	MINIMUM_VELODYNE_DISTANCE 0.75
#define	MINIMUM_HEIGHT_DIFFERENCE 	0.07

#define GLOBAL_POS_QUEUE_SIZE		100

/**
  * Prototypes
  */

void localize_using_map_merge_online_map_with_offline_map(carmen_map_t *offline_map);
int  localize_using_map_velodyne_variable_scan(int sensor_number, carmen_velodyne_variable_scan_message *message);
int  localize_using_map_velodyne_partial_scan(carmen_velodyne_partial_scan_message *velodyne_message);
void localize_using_map_publish_map(double timestamp);
void localize_using_map_set_robot_pose_into_the_map(double v, double phi, double timestamp);
void localize_using_map_update_grid_map(carmen_point_t xt, double *zt, sensor_parameters_t *sensor_params);
void localize_using_map_save_current_map();

void localize_using_map_initialize(carmen_map_config_t *main_map_config);
void localize_using_map_change_map_origin_to_another_map_block(carmen_position_t *map_origin);

#endif /* LOCALIZE_USING_MAP_H_ */

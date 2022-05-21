#ifndef MAPPER_H_
#define MAPPER_H_

#include <carmen/moving_objects_interface.h>
#include "neural_mapper_io.h"


//#define	MINIMUM_VELODYNE_DISTANCE 0.75
#define	MINIMUM_HEIGHT_DIFFERENCE 	0.07
//#define STEREO_MAPPING_SENSOR_INDEX 	10

#define NUM_VELODYNE_POINT_CLOUDS	5
#define GLOBAL_POS_QUEUE_SIZE		100

#define NUMBER_OF_SENSORS 25      // The number_of_sensors must be the maximun number of sensors: 25 

#define USE_REAR_BULLBAR

typedef struct
{
	carmen_map_t *offline_map;

	carmen_map_t *occupancy_map;
	carmen_map_t *sum_occupancy_map;
	carmen_map_t *count_occupancy_map;

	carmen_map_t *new_occupancy_map;
	carmen_map_t *new_sum_occupancy_map;
	carmen_map_t *new_count_occupancy_map;

	carmen_map_t *sum_remission_map;
	carmen_map_t *sum_sqr_remission_map;
	carmen_map_t *count_remission_map;

	carmen_map_t *new_sum_remission_map;
	carmen_map_t *new_sum_sqr_remission_map;
	carmen_map_t *new_count_remission_map;

	carmen_map_t *snapshot_map;
	carmen_map_t *sum_remission_snapshot_map;
	carmen_map_t *sum_sqr_remission_snapshot_map;
	carmen_map_t *count_remission_snapshot_map;

	carmen_map_t *log_odds_snapshot_map;

	carmen_map_t *moving_objects_raw_map;
} carmen_map_set_t;


void mapper_merge_online_map_with_offline_map(carmen_map_set_t *map_set);

int mapper_stereo_velodyne_variable_scan(int sensor_number, carmen_velodyne_variable_scan_message *message);

int update_data_params_with_lidar_data(int sensor_number, carmen_velodyne_variable_scan_message *msg);

int mapper_velodyne_partial_scan(int sensor_number, carmen_velodyne_partial_scan_message *velodyne_message);

void mapper_publish_map(double timestamp);

int run_snapshot_mapper(carmen_map_set_t *map_set);

void add_virtual_laser_points(carmen_map_t *map, carmen_mapper_virtual_laser_message *virtual_laser_message);

void add_moving_objects(carmen_map_t *map, carmen_moving_objects_point_clouds_message *moving_objects_message);

void mapper_set_robot_pose_into_the_map(carmen_map_set_t *map_set, carmen_localize_ackerman_globalpos_message *globalpos_message, int UPDATE_CELLS_BELOW_CAR);

void mapper_update_grid_map(carmen_map_set_t *map_set, carmen_point_t xt, double *zt, sensor_parameters_t *sensor_params);

void mapper_save_current_map(carmen_map_set_t *map_set);

void mapper_periodically_save_current_map(carmen_map_set_t *map_set, double timestamp);

carmen_map_set_t *mapper_initialize(carmen_map_config_t *map_config, carmen_robot_ackerman_config_t main_car_config, bool use_merge_between_maps);

void mapper_change_map_origin_to_another_map_block(char *map_path, carmen_map_set_t *map_set,
		carmen_position_t *map_origin, bool save_map, bool force_saving_new_map = false);

void mapper_change_map_origin_to_another_map_block_with_clones(char *map_path, carmen_map_set_t *map_set,
		carmen_position_t *map_origin, bool save_map, bool force_saving_new_map = false);

int run_mapper(carmen_map_set_t *map_set, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global);

int run_mapper_with_remision_threshold(carmen_map_set_t *map_set, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global, double rays_threshold_to_merge_between_maps);

void map_decay_to_offline_map(carmen_map_set_t *map_set);

carmen_map_t *get_the_map();

void set_rays_threshold_to_merge_between_maps();

double get_rays_threshold_to_merge_between_maps();

void carmen_mapper_initialize_transforms();

void carmen_mapper_get_highest_sensor();

void carmen_mapper_sensors_params_handler(char *module, char *variable, __attribute__((unused)) char *value);

void carmen_mapper_read_alive_lidars_configs(int argc, char **argv);

void carmen_mapper_get_alive_sensors(int argc, char **argv);

void read_parameters_semi_trailer(int semi_trailer_type);

int *carmen_mapper_generates_ray_order(int size);

void carmen_mapper_get_sensors_param(int argc, char **argv);

void carmen_mapper_sort_ray_order_by_vertical_correction_angles(sensor_parameters_t params);

void carmen_mapper_sort_vertical_correction_angles(sensor_parameters_t params);

void carmen_mapper_get_lidars_sensor_params();

void carmen_mapper_init_velodyne_points(spherical_point_cloud **velodyne_points_out, unsigned char ***intencity, carmen_pose_3D_t **robot_pose_out,
		carmen_vector_3D_t **robot_velocity_out, double **robot_timestamp_out, double **robot_phi_out, double **points_timestamp_out);

void carmen_mapper_override_mapping_mode_params(int argc, char **argv);

void get_sensors_param_pose_handler(__attribute__((unused)) char *module, __attribute__((unused)) char *variable, __attribute__((unused)) char *value);

void carmen_mapper_read_parameters(int argc, char **argv, carmen_map_config_t *map_config, carmen_robot_ackerman_config_t *p_car_config);

void update_log_odds_of_cells_in_the_velodyne_perceptual_field(carmen_map_set_t *map_set, sensor_parameters_t *sensor_params,
		sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global, int point_cloud_index, int update_cells_crossed_by_rays,
		int build_snapshot_map __attribute__ ((unused)));

void build_sensor_point_cloud(spherical_point_cloud **points, unsigned char **intensity, int *point_cloud_index, int num_points, int max_point_buffer,
		int use_remission);

void carmen_mapper_update_cells_bellow_robot(carmen_point_t pose, carmen_map_t *map, double prob);

carmen_map_set_t *get_a_map_set(carmen_map_config_t map_config, bool use_merge_between_maps, bool create_map_sum_and_count, bool use_remission);


#endif /* MAPPER_H_ */

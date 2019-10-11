#ifndef CARMEN_PROB_MAP_H
#define CARMEN_PROB_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <carmen/carmen.h>
#include <stdbool.h>
#include "prob_measurement_model.h"

#define      DISTANCE_MAP_HUGE_DISTANCE     20
#define      HUGE_DISTANCE     				32000

typedef enum _ProbabilisticMapColor
{
	PMC_BLACK = 0,
	PMC_WHITE = 255,
	PMC_UNKNOWN_AREA = 128,
	PMC_OBSTACLE_COLOR_LIMIT = 100,
	PMC_INFLATING_COLOR = PMC_OBSTACLE_COLOR_LIMIT - 1,
	PMC_RED = PMC_OBSTACLE_COLOR_LIMIT + 1,
	PMC_GREEN = PMC_RED+1,
	PMC_BLUE = PMC_GREEN+1,
	PMC_GRAY = PMC_BLUE+1
} ProbabilisticMapColor;

typedef struct _ProbabilisticMapParams
{
	double width;
	double height;
	double range_start;
	double range_step;
	double range_max;
	double range_factor;	// by how much a laser beam measurament must be divided to put it in meters
	int num_ranges;		// number of laser beams in a LIDAR sweep

	double grid_res;
	int grid_sx, grid_sy, grid_size;

	int l0, locc, lfree;

	int log_odds_max;
	int log_odds_min;
	int log_odds_bias;
	
	int robot_is_rectangular;
	double robot_width;
	double robot_length;
	double robot_vertical_displacement_from_center;
} ProbabilisticMapParams;

typedef struct _ProbabilisticMap
{
	float **log_odds_map;
	int **image_map;
} ProbabilisticMap;

typedef struct 
{
	int x;
	int y;
	int width;
	int height;
} ProbabilisticMapBoundary;


typedef struct log_odds_param
{
	double log_odds_free;
	double log_odds_occ;
	double log_odds_l0;
} log_odds_param;


typedef struct _sensor_parameters
{
	int alive;
	SENSOR_TYPE sensor_type;
	double range_max, range_max_factor, current_range_max;
	double height;
	log_odds_param log_odds;
	char *name;
	double unexpeted_delta_range_sigma;
	double *remission_calibration;
	double *vertical_correction;
	double *delta_difference_mean, *delta_difference_stddev;
	int vertical_resolution;
	int *ray_order;
	double unsafe_height_above_ground;

	double sigma_zhit, zhit, zshort, zmax, zrand;
	double lambda_short;
	double fov_range;
	double start_angle;
	int sampling_step;
	int laser_beams;
	double front_offset;
	double side_offset;
	double angular_offset;
	double time_spent_by_each_scan;

	carmen_pose_3D_t pose;
	carmen_pose_3D_t sensor_support_pose;
	rotation_matrix *sensor_to_support_matrix;
	rotation_matrix *support_to_car_matrix;
	carmen_vector_3D_t sensor_robot_reference;

	int use_remission;

	double cutoff_negative_acceleration;

	FILE *save_calibration_file;
	float ***calibration_table;
} sensor_parameters_t;


typedef struct _sensor_data
{
	int vectors_size;
	double last_timestamp;
	double current_timestamp;
	int point_cloud_index;
	unsigned char **intensity;
	spherical_point_cloud *points;
	double *points_timestamp;
	carmen_pose_3D_t *robot_pose;
	carmen_vector_3D_t *robot_velocity;
	double *robot_phi;
	double *robot_timestamp;
	carmen_vector_2D_t **ray_position_in_the_floor, **ray_origin_in_the_floor;
	double **ray_size_in_the_floor;
	double **processed_intensity;
	double **obstacle_height;
	double **occupancy_log_odds_of_each_ray_target;
	int **maxed;
	int **ray_hit_the_robot;
	int *ray_that_hit_the_nearest_target;
} sensor_data_t;


typedef struct
{
	carmen_map_config_t config;
	short int *complete_x_offset;
	short int *complete_y_offset;
	short int **x_offset;
	short int **y_offset;
	double *complete_distance;
	double **distance;
} carmen_prob_models_distance_map;


void init_carmen_map(const ProbabilisticMapParams *params, carmen_map_t *carmen_map);
void init_probabilistic_grid_map_model(ProbabilisticMapParams *params, carmen_map_t *carmen_map);
void init_probabilistic_map(ProbabilisticMapParams *params, carmen_map_t *carmen_map, ProbabilisticMap *map, int num_particles);
//void load_probabilistic_map(ProbabilisticMapParams *params, carmen_map_t *carmen_map, ProbabilisticMap *map, int num_particles);
void load_probabilistic_map_slam(ProbabilisticMapParams *params, carmen_map_t *carmen_map, ProbabilisticMap * map, int num_particles);
void copy_probabilistic_map_to_carmen_map(const ProbabilisticMapParams *params, carmen_map_t *carmen_map, ProbabilisticMap *probabilistic_map);
void copy_carmen_map_to_probabilistic_map(const ProbabilisticMapParams *params, carmen_map_t *carmen_map, ProbabilisticMap *probabilistic_map);
void copy_probabilistic_map(ProbabilisticMap *dst, const ProbabilisticMap *src);
void copy_complete_map_to_map(carmen_map_p carmen_map, carmen_map_config_t config);
void move_probabilistic_map(ProbabilisticMap *map, carmen_point_t pose);
void save_probabilistic_map(const ProbabilisticMap *map, carmen_point_t pose);
double carmen_ray_cast(carmen_point_t xt, int sample_index, carmen_map_t *map, double range_max, double start_angle, double angle_step);
double ray_cast(carmen_point_t xt, int sample_index, const ProbabilisticMap *map, ProbabilisticMapColor color);
double ray_cast_unknown(carmen_point_t xt, const ProbabilisticMap *map, double max_distance);
double ray_cast_unknown_between_coordinates(double ax, double ay, double bx, double by, const ProbabilisticMap *map);
double ray_cast_between_coordinates(double ax, double ay, double bx, double by, const ProbabilisticMap *map, ProbabilisticMapColor color);
void draw_ray_cast(carmen_point_t xt, double distance, double direction, const ProbabilisticMap *map, ProbabilisticMapColor color);
void draw_ray(double ax, double ay, double bx, double by, const ProbabilisticMap *map, ProbabilisticMapColor color);
void update_cells_above_robot(const ProbabilisticMap *map, carmen_point_t xt, double robot_length, double robot_width, ProbabilisticMapColor color);
void carmen_update_cells_in_the_sensor_perceptual_field(carmen_map_t *map, carmen_point_t xt, const double *zt, sensor_parameters_t *sensor_params);
void carmen_update_cells_in_the_laser_perceptual_field(carmen_map_t *map, carmen_point_t xt, const double *zt, BeanRangeFinderMeasurementModelParams *laser_params);
void carmen_update_cells_below_robot(carmen_map_t *map, carmen_point_t xt);
void occupancy_grid_mapping(int xi, int yi, const ProbabilisticMap *map, int inverse_sensor_model_value);
double get_log_odds_map_cell(const carmen_map_t *map, int x, int y);
void set_log_odds_map_cell(carmen_map_t *map, int x, int y, double lt_i);
double get_log_odds(double p_mi);
double carmen_prob_models_log_odds_to_probabilistic(double lt_i);
double carmen_prob_models_probabilistic_to_log_odds(double p_mi);
void set_image_map_cell(const ProbabilisticMap *map, int x, int y, int value);
double grid_to_map_x(int x);
double grid_to_map_y(int y);
int map_to_grid_x(double x);
int map_to_grid_y(double y);
//int map_to_grid_x(carmen_map_t *map, double x);
//int map_to_grid_y(carmen_map_t *map, double y);
int map_grid_offset_x();
int map_grid_offset_y();
void inflate_map(const ProbabilisticMap *map, carmen_point_t robotPose, double robot_length, double robot_width);
void inflate_map_and_robot(const ProbabilisticMap *map, carmen_point_t robotPose, double robot_length, double robot_width);
void get_uniform_random_free_cells(carmen_point_t *random_cells_of_given_color, int M, const ProbabilisticMap *map, const ProbabilisticMapBoundary *map_boundary);
void translate_map(const ProbabilisticMap *map, ProbabilisticMapParams map_config, double dx, double dy);
void update_cells_in_the_camera_perceptual_field(const carmen_point_t Xt, const ProbabilisticMap *global_map, ProbabilisticMapParams global_map_params, ProbabilisticMapParams stereo_map_params, double y_offset, float *zt);
void copy_probabilistic_map_to_image_buffer(ProbabilisticMapParams map_config, ProbabilisticMap *probabilistic_map, unsigned char *dst, int n_channels);
double bearing(int i, double theta);
int map_grid_is_valid(carmen_map_t* map, int x, int y);

#ifdef __cplusplus
int carmen_prob_models_unaceptable_height(double obstacle_height, double highest_sensor, double safe_range_above_sensors, double safe_height_from_ground = -20.0);
#endif

double carmen_prob_models_compute_expected_delta_ray(double ray_size, int ray_index, double *vertical_correction, double sensor_height);

#ifdef __cplusplus
void
carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(sensor_data_t *sensor_data, sensor_parameters_t *sensor_params, int scan_index,
		double highest_sensor, double safe_range_above_sensors, int reduce_sensitivity, int thread_id, double safe_height_from_ground = -20.0);
#endif

void
carmen_prob_models_get_occuppancy_log_odds_by_height(sensor_data_t *sensor_data, sensor_parameters_t *sensor_params, int scan_index,
		double highest_sensor, double safe_range_above_sensors, int reduce_sensitivity, int thread_id);

void
carmen_prob_models_compute_relevant_map_coordinates(sensor_data_t *sensor_data, sensor_parameters_t *sensor_params, int scan_index,
		carmen_vector_3D_t robot_position, carmen_pose_3D_t sensor_board_pose, rotation_matrix *r_matrix_robot_to_global, rotation_matrix *board_to_robot_matrix,
		double robot_wheel_radius, double x_origin, double y_origin, carmen_robot_ackerman_config_t *car_config,
		int overwrite_blind_spots_around_the_robot, int thread_id);

void
carmen_prob_models_compute_relevant_map_coordinates_with_remission_check(sensor_data_t *sensor_data, sensor_parameters_t *sensor_params, int scan_index,
		carmen_vector_3D_t robot_position, carmen_pose_3D_t sensor_board_pose, rotation_matrix *r_matrix_robot_to_global, rotation_matrix *board_to_robot_matrix,
		double robot_wheel_radius, double x_origin, double y_origin, carmen_robot_ackerman_config_t *car_config,
		int overwrite_blind_spots_around_the_robot, int thread_id, int use_remission);

void carmen_prob_models_updade_cells_bellow_robot(carmen_point_t pose, carmen_map_t *map, double prob, carmen_robot_ackerman_config_t *car_config);
void carmen_prob_models_alloc_sensor_data(sensor_data_t *sensor_data, int vertical_resolution, int number_of_threads);
void carmen_prob_models_log_odds_occupancy_grid_mapping(carmen_map_t *map, int xi, int yi, double inverse_sensor_model_value, ProbabilisticMapParams map_params);

#ifdef __cplusplus
void carmen_prob_models_update_log_odds_of_cells_hit_by_rays(carmen_map_t *map,  sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, double highest_sensor, double safe_range_above_sensors, int thread_id, double safe_height_from_ground = -20.0);
void carmen_prob_models_update_sum_and_count_of_cells_hit_by_rays(carmen_map_t *map, carmen_map_t *sum_occupancy_map, carmen_map_t *count_occupancy_map,  sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, double highest_sensor, double safe_range_above_sensors, int thread_id, double safe_height_from_ground = -20.0);
#endif

void carmen_prob_models_update_log_odds_of_cells_crossed_by_ray(carmen_map_t *log_odds_map, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int thread_id);
void carmen_prob_models_update_log_odds_of_nearest_target(carmen_map_t *map,  sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, double highest_sensor, double safe_range_above_sensors, int thread_id);
void carmen_prob_models_get_maximum_probability_of_cells_hit_by_rays(carmen_map_t *map,  sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int thread_id);
void carmen_prob_models_update_log_odds_of_cells_hit_by_ldmrs_rays(carmen_map_t *map,  sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int thread_id);
void carmen_prob_models_update_cells_crossed_by_ray(carmen_map_t *map, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int thread_id);
void carmen_prob_models_update_sum_and_count_cells_crossed_by_ray(carmen_map_t *map, carmen_map_t *sum_occupancy_map, carmen_map_t *count_occupancy_map, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int thread_id);

void carmen_prob_models_save_compact_map_as_binary_file(carmen_compact_map_t *cmap, char *path);
void carmen_prob_models_create_compact_map(carmen_compact_map_t *cmap, carmen_map_t *map, double value);
void carmen_prob_models_free_compact_map(carmen_compact_map_t *map);
void carmen_prob_models_uncompress_compact_map(carmen_map_t *map, carmen_compact_map_t *cmap);

void carmen_prob_models_clear_carmen_map_using_compact_map(carmen_map_t *map, carmen_compact_map_t *cmap, double value);


int carmen_prob_models_ray_hit_the_robot(double distance_between_rear_robot_and_rear_wheels , double robot_length, double robot_width, double x, double y);

carmen_map_t *carmen_prob_models_check_if_new_snapshot_map_allocation_is_needed(carmen_map_t *snapshot_map, carmen_map_t *current_map);
carmen_map_t *carmen_prob_models_check_if_new_log_odds_snapshot_map_allocation_is_needed(carmen_map_t *log_odds_snapshot_map, carmen_map_t *current_map);
void carmen_prob_models_update_current_map_with_snapshot_map_and_clear_snapshot_map(carmen_map_t *current_map, carmen_map_t *snapshot_map);
void carmen_prob_models_update_current_map_with_log_odds_snapshot_map_and_clear_snapshot_map(carmen_map_t *current_map, carmen_map_t *log_odds_snapshot_map, double log_odds_l0);
void carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(carmen_map_t *current_map, carmen_map_t *snapshot_map);
void carmen_prob_models_overwrite_current_map_with_log_odds_snapshot_map_and_clear_snapshot_map(carmen_map_t *current_log_odds_map, carmen_map_t *log_odds_snapshot_map, double log_odds_l0);
void carmen_prob_models_clear_cells_hit_by_single_ray(carmen_map_t *log_odds_snapshot_map, double log_odds_occ, double log_odds_l0);

#ifdef __cplusplus
void carmen_prob_models_update_intensity_of_cells_hit_by_rays(carmen_map_t *sum_remission_map,
		carmen_map_t *sum_sqr_remission_map, carmen_map_t *count_remission_map,
		sensor_parameters_t *sensor_params,
		sensor_data_t *sensor_data, double highest_sensor, double safe_range_above_sensors, cell_coords_t *map_cells_hit_by_each_rays, int thread_id, double safe_height_from_ground = -20.0);
#endif

void carmen_prob_models_add_intensity_of_cells_hit_by_rays(carmen_compact_map_t *mean_remission_compact_map,
		sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, double highest_sensor,
		double safe_range_above_sensors, cell_coords_t *map_cells_hit_by_each_rays, int thread_id);

void carmen_prob_models_calc_mean_and_variance_remission_map(carmen_map_t *mean_remission_map, carmen_map_t *variance_remission_map, carmen_map_t *sum_remission_map, carmen_map_t *sum_sqr_remission_map, carmen_map_t *count_remission_map);
void carmen_prob_models_calc_mean_remission_map(carmen_map_t *mean_remission_map, carmen_map_t *sum_remission_map, carmen_map_t *count_remission_map);

void carmen_prob_models_update_intensity_of_cells_hit_by_rays_for_calibration(carmen_map_t *sum_remission_map, carmen_map_t *sum_sqr_remission_map, carmen_map_t *count_remission_map, carmen_map_t *remission_map, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, double highest_sensor, double safe_range_above_sensors, int thread_id);

void carmen_prob_models_clean_carmen_map(carmen_map_t *map);

void carmen_prob_models_convert_obstacles_map_to_cost_map(carmen_map_t *cost_map, carmen_map_t *map, double occupancy_threshold,
		double distance_for_zero_cost_in_meters, int invert_map);

void carmen_prob_models_initialize_cost_map(carmen_map_t *cost_map, carmen_map_config_t config, double resolution);

void carmen_prob_models_build_obstacle_cost_map(carmen_map_t *cost_map, carmen_map_t *map, double resolution, double obstacle_cost_distance, double occupancy_threshold);

void carmen_prob_models_create_distance_map(carmen_prob_models_distance_map *lmap, carmen_map_p map,
		double minimum_occupied_prob);

/* verify if a given point is inside a given ellipse */
int is_inside_ellipse(int dx, int dy, double i_minor, double i_major, double cosalpha, double sinalpha);

void carmen_prob_models_create_masked_distance_map(carmen_prob_models_distance_map *lmap, carmen_map_p map,
		double minimum_occupied_prob, carmen_point_p robot_position, carmen_point_p goal_position);

void carmen_prob_models_initialize_distance_map(carmen_prob_models_distance_map *lmap, carmen_map_config_t config);

float ***load_calibration_table(char *calibration_file);

int get_distance_index(double distance);

#ifdef __cplusplus
}
#endif

#endif

#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <carmen/grid_mapping.h>
#include <carmen/mapper_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/global_graphics.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/stereo_velodyne_interface.h>
#include <omp.h>

#include <iostream>
#include <fstream>

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#include "mapper.h"

#define	UPDATE_CELLS_CROSSED_BY_RAYS		1
#define	DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS	0

#define HUGE_DISTANCE     32000

#define MAX_VIRTUAL_LASER_SAMPLES 10000

extern double safe_range_above_sensors;
extern double safe_height_from_ground;
extern int level_msg;
extern double safe_height_from_ground_level;

int clean_map_bellow_car = 0;

extern double robot_wheel_radius;

extern double highest_sensor;

extern int update_and_merge_with_mapper_saved_maps;
extern int build_snapshot_map;
extern int update_and_merge_with_snapshot_map;
extern int decay_to_offline_map;
extern int create_map_sum_and_count;
extern int use_remission;

extern sensor_parameters_t *sensors_params;
extern sensor_data_t *sensors_data;
extern int number_of_sensors;
carmen_pose_3D_t front_bullbar_pose;
carmen_pose_3D_t rear_bullbar_pose;
carmen_pose_3D_t laser_ldmrs_pose;
double mapper_unsafe_height_above_ground;
int publish_moving_objects_raw_map;
int merge_with_offline_map;
int update_cells_below_car;
int mapper_save_map = 1;
double rays_threshold_to_merge_between_maps = 1e12;
int visual_odometry_is_global_pos = 0;
sensor_parameters_t ultrasonic_sensor_params;
carmen_pose_3D_t ultrasonic_sensor_r1_g;
carmen_pose_3D_t ultrasonic_sensor_r2_g;
carmen_pose_3D_t ultrasonic_sensor_l1_g;
carmen_pose_3D_t ultrasonic_sensor_l2_g;
int use_truepos = 0;
carmen_lidar_config lidar_config[MAX_NUMBER_OF_LIDARS];
int g_argc;
char **g_argv;
char *calibration_file = NULL;
char *save_calibration_file = NULL;
int neural_map_num_clouds = 1;
extern int mapping_mode;
int publish_diff_map = 0;
double publish_diff_map_interval = 0.5;
double mapper_velodyne_range_max;
double mapper_range_max_factor = 1.0;
bool use_merge_between_maps = false;
tf::Transformer tf_transformer(false);
static carmen_pose_3D_t car_pose_g;
static carmen_pose_3D_t camera_pose_g;

extern char *map_path;

extern int robot_near_strong_slow_down_annotation;
extern int ok_to_publish;
extern int number_of_threads;

extern int use_unity_simulator;
int neural_mapper_initialized = 0;

extern carmen_semi_trailers_config_t semi_trailer_config;
extern carmen_pose_3D_t sensor_board_1_pose;
extern carmen_pose_3D_t velodyne_pose;

static carmen_pose_3D_t gps_pose_in_the_car;


/*************************************
* Variables for Neural mapper dataset generation maps
*/
extern int generate_neural_mapper_dataset;
extern int use_neural_mapper;
extern int neural_mapper_max_distance_meters;
//a cada neural_mapper_data_pace uma amostra do neural mapper sera gerada
extern int neural_mapper_data_pace;
static carmen_position_t neural_mapper_car_position_according_to_map;
char neural_mapper_dataset_path[1024] = "/media/vinicius/NewHD/neural_mapper_raw/guarapari-20170403-2_no_bumblebee/";

/***********************************/

extern rotation_matrix *r_matrix_car_to_global;

int globalpos_initialized = 0;
extern carmen_localize_ackerman_globalpos_message *globalpos_history;
extern int last_globalpos;

extern carmen_robot_ackerman_config_t car_config;
extern carmen_map_config_t map_config;

static carmen_laser_laser_message flaser; // moving_objects

extern carmen_rddf_annotation_message last_rddf_annotation_message;

carmen_mapper_virtual_laser_message virtual_laser_message;

carmen_mapper_virtual_scan_message virtual_scan_message;

carmen_moving_objects_point_clouds_message *moving_objects_message = NULL;

extern bool offline_map_available;
extern double time_secs_between_map_save;

double min_force_obstacle_height = 100.0;
double max_force_obstacle_height = -100.0;


void
change_sensor_rear_range_max(sensor_parameters_t *sensor_params, double angle)
{
	if (clean_map_bellow_car)
	{
		sensor_params->current_range_max = 0.0;
		return;
	}

	if ((angle > M_PI / 2.0) || (angle < -M_PI / 2.0))
			sensor_params->current_range_max = sensor_params->range_max / sensor_params->range_max_factor;
	else
		sensor_params->current_range_max = sensor_params->range_max;
}


void
build_front_laser_message_from_velodyne_point_cloud(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, spherical_point_cloud v_zt, int i)
{
	static int first = 1;

	if (first)
	{
		flaser.host = carmen_get_host();
		flaser.num_readings = 720;
		flaser.range = (double *)calloc
				(720, sizeof(double));
		carmen_test_alloc(flaser.range);

		flaser.num_remissions = 0;
		flaser.remission = 0;

		flaser.config.angular_resolution = 0.5;
		flaser.config.fov = sensor_params->fov_range;
		flaser.config.maximum_range = sensor_params->range_max;
		flaser.config.remission_mode = REMISSION_NONE;
		flaser.config.start_angle = sensor_params->start_angle;
		first = 0;
	}

	int laser_ray_angle_index = 0;
	laser_ray_angle_index = (int)(v_zt.sphere_points[i].horizontal_angle / 0.5) % (720);

	//if (carmen_prob_models_log_odds_to_probabilistic(sensor_data->occupancy_log_odds_of_each_ray_target[sensor_data->ray_that_hit_the_nearest_target]) > 0.95)
	flaser.range[laser_ray_angle_index] = sensor_data->ray_size_in_the_floor[0][sensor_data->ray_that_hit_the_nearest_target[0]];

	if (sensor_data->maxed[0][sensor_data->ray_that_hit_the_nearest_target[0]])
			flaser.range[laser_ray_angle_index] = sensor_params->current_range_max;

//	printf("%lf ", flaser.range[laser_ray_angle_index]);

}


void
compute_virtual_scan_point(int ray_id, int tid, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data,
		carmen_map_set_t *map_set, bool is_ldmrs = false)
{
	if (!sensor_data->maxed[tid][ray_id])
	{
		int sensor = virtual_scan_message.num_sensors - 1;
		int ray_num = virtual_scan_message.virtual_scan_sensor[sensor].num_points;

		cell_coords_t cell_hit_by_ray;

		double x = sensor_data->ray_position_in_the_floor[tid][ray_id].x;
		double y = sensor_data->ray_position_in_the_floor[tid][ray_id].y;

		cell_hit_by_ray.x = round(x / map_set->log_odds_snapshot_map->config.resolution);
		cell_hit_by_ray.y = round(y / map_set->log_odds_snapshot_map->config.resolution);

		if (map_grid_is_valid(map_set->log_odds_snapshot_map, cell_hit_by_ray.x, cell_hit_by_ray.y) &&
			((sensor_data->occupancy_log_odds_of_each_ray_target[tid][ray_id] > sensor_params->log_odds.log_odds_occ / 10.0) || is_ldmrs) &&
			(map_set->offline_map->map[cell_hit_by_ray.x][cell_hit_by_ray.y] <= 0.5))
		{
			virtual_scan_message.virtual_scan_sensor[sensor].points[ray_num].x = x + map_set->x_origin;
			virtual_scan_message.virtual_scan_sensor[sensor].points[ray_num].y = y + map_set->y_origin;
			virtual_scan_message.virtual_scan_sensor[sensor].num_points = ray_num + 1;
		}
	}
}


void
build_virtual_scan_message_ldmrs(int tid, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, carmen_map_set_t *map_set)
{
	double min_ray_size_in_the_floor = 1000.0;
	int nearest_target = 0;

	for (int k = 0; k < sensor_params->vertical_resolution; k++)
	{
		double ray_size_in_the_floor = sensor_data->ray_size_in_the_floor[tid][k];

		if (ray_size_in_the_floor > min_ray_size_in_the_floor)
		{
			ray_size_in_the_floor = min_ray_size_in_the_floor;
			nearest_target = k;
		}
	}
	compute_virtual_scan_point(nearest_target, tid, sensor_params, sensor_data, map_set, true);
}


void
build_virtual_scan_message_velodyne(int tid, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, carmen_map_set_t *map_set)
{
	int k = sensor_data->ray_that_hit_the_nearest_target[tid];
	compute_virtual_scan_point(k, tid, sensor_params, sensor_data, map_set);
}


void
initialize_virtual_scan_message_update(int sensor_id, carmen_pose_3D_t robot_pose, carmen_pose_3D_t robot_interpolated_position,
		double time_spent_in_the_entire_sensor_sweep, double last_sensor_angle, double v, double phi,
		int num_points, int point_cloud_index, rotation_matrix *r_matrix_robot_to_global, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data)
{
	r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_pose.orientation);
	carmen_vector_3D_t sensor_position_in_the_world = carmen_change_sensor_reference(robot_interpolated_position.position,
			sensor_params->sensor_robot_reference, r_matrix_robot_to_global);

	int n = virtual_scan_message.num_sensors;
	virtual_scan_message.virtual_scan_sensor = (carmen_virtual_scan_sensor_t *) realloc(virtual_scan_message.virtual_scan_sensor, (n + 1) * sizeof(carmen_virtual_scan_sensor_t));

	carmen_point_t global_pos = {robot_pose.position.x, robot_pose.position.y, robot_pose.orientation.yaw};
	carmen_point_t sensor_pos = {sensor_position_in_the_world.x, sensor_position_in_the_world.y, robot_interpolated_position.orientation.yaw};
	virtual_scan_message.virtual_scan_sensor[n].sensor_pos = sensor_pos;
	virtual_scan_message.virtual_scan_sensor[n].global_pos = global_pos;
	virtual_scan_message.virtual_scan_sensor[n].sensor_id = sensor_id;
	virtual_scan_message.virtual_scan_sensor[n].time_spent_in_the_entire_sensor_sweep = time_spent_in_the_entire_sensor_sweep;
	virtual_scan_message.virtual_scan_sensor[n].last_sensor_angle = last_sensor_angle;
	virtual_scan_message.virtual_scan_sensor[n].v = v;
	virtual_scan_message.virtual_scan_sensor[n].w = tan(phi) * (v / car_config.distance_between_front_and_rear_axles);
	virtual_scan_message.virtual_scan_sensor[n].num_points = 0; // O total de pontos alocados abaixo pode nao ser totalmente alocado, pois os pontos realmente usados sao preenchidos sob demanda;
	virtual_scan_message.virtual_scan_sensor[n].points = (carmen_point_t *) malloc(num_points * sizeof(carmen_point_t));
	virtual_scan_message.virtual_scan_sensor[n].timestamp = sensor_data->points_timestamp[point_cloud_index];

	virtual_scan_message.num_sensors = n + 1;
}


//FILE *plot_data;
static bool
mapper_map_configs_are_different(carmen_map_config_t &a, carmen_map_config_t &b)
{
	return (a.x_origin != b.x_origin || a.y_origin != b.y_origin || a.x_size != b.x_size || a.y_size != b.y_size || a.resolution != b.resolution);
}


static void
mapper_merge_snapshot_map(carmen_map_t *count_remission_map, carmen_map_t *source, carmen_map_t *target, double rays_threshold_to_merge_between_maps)
{
	for (int i = 0; i < count_remission_map->config.x_size * count_remission_map->config.y_size; i++)
	{
		if (rays_threshold_to_merge_between_maps > count_remission_map->complete_map[i])
			target->complete_map[i] = source->complete_map[i];
	}
}


static void
mapper_clone_map(carmen_map_t *source, carmen_map_t *target, char map_type)
{
	if (mapper_map_configs_are_different(source->config, target->config))
	{
		free(target->complete_map);
		free(target->map);
		carmen_grid_mapping_initialize_map(target, source->config.x_size, source->config.resolution, map_type);
	}

	for (int i = 0; i < source->config.x_size * source->config.y_size; ++i)
		target->complete_map[i] = source->complete_map[i];
}


static void
update_log_odds_of_cells_in_the_velodyne_perceptual_field_with_snapshot_maps(
	carmen_map_set_t *map_set,
	sensor_parameters_t *sensor_params,
	sensor_data_t *sensor_data,
	rotation_matrix *r_matrix_robot_to_global,
	int update_cells_crossed_by_rays
)
{
	int tid = omp_get_thread_num();
	int point_cloud_index = sensor_data->point_cloud_index;

	spherical_point_cloud v_zt = sensor_data->points[point_cloud_index];
	int N = v_zt.num_points / sensor_params->vertical_resolution;

	double v = sensor_data->robot_velocity[point_cloud_index].x;
	double phi = sensor_data->robot_phi[point_cloud_index];

	double dt = sensor_params->time_spent_by_each_scan;
	double dt1 = sensor_data->points_timestamp[point_cloud_index] - sensor_data->robot_timestamp[point_cloud_index] - (double) N * dt;
	carmen_pose_3D_t robot_interpolated_position = sensor_data->robot_pose[point_cloud_index];
	int i = 0;

	// Ray-trace the grid
	carmen_pose_3D_t robot_pose = sensor_data->robot_pose[point_cloud_index];

	initialize_virtual_scan_message_update(
		VELODYNE,
		robot_pose,
		robot_interpolated_position,
		dt * (double) N,
		v_zt.sphere_points[sensor_params->vertical_resolution * (N - 1)].horizontal_angle,
		v,
		phi,
		N,
		point_cloud_index,
		r_matrix_robot_to_global,
		sensor_params,
		sensor_data
	);

	for (int j = 0; j < N; j += 1)
	{
		i = j * sensor_params->vertical_resolution;
		double dt2 = j * dt;
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(
			robot_pose,
			dt1 + dt2,
			v,
			phi,
			car_config.distance_between_front_and_rear_axles
		);

		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

		change_sensor_rear_range_max(sensor_params, v_zt.sphere_points[i].horizontal_angle);

		carmen_prob_models_compute_relevant_map_coordinates_with_remission_check(
			sensor_data,
			sensor_params,
			i,
			robot_interpolated_position.position,
			sensor_params->sensor_support_pose,
			r_matrix_robot_to_global,
			sensor_params->support_to_car_matrix,
			robot_wheel_radius,
			map_set->x_origin,
			map_set->y_origin,
			&car_config,
			robot_near_strong_slow_down_annotation,
			tid,
			use_remission
		);

		if (use_neural_mapper)
			neural_mapper_update_input_maps(sensor_data, sensor_params, tid, map_set->log_odds_snapshot_map, map_config, map_set->x_origin, map_set->y_origin, highest_sensor, safe_range_above_sensors);

		carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(
			sensor_data,
			sensor_params,
			i,
			highest_sensor,
			safe_range_above_sensors,
			robot_near_strong_slow_down_annotation,
			tid
		);

		build_virtual_scan_message_velodyne(tid, sensor_params, sensor_data, map_set);

		if (update_cells_crossed_by_rays == UPDATE_CELLS_CROSSED_BY_RAYS)
		{
			if (create_map_sum_and_count)
				carmen_prob_models_update_sum_and_count_cells_crossed_by_ray(
					map_set->snapshot_map,
					map_set->sum_occupancy_map,
					map_set->count_occupancy_map, sensor_params, sensor_data, tid
				);
			else
				carmen_prob_models_update_cells_crossed_by_ray(map_set->snapshot_map, sensor_params, sensor_data, tid);
		}

		if (create_map_sum_and_count)
			carmen_prob_models_update_sum_and_count_of_cells_hit_by_rays_into_log_odds_snapshot_map(
				map_set->log_odds_snapshot_map,
				map_set->sum_occupancy_map,
				map_set->count_occupancy_map,
				sensor_params,
				sensor_data,
				highest_sensor,
				safe_range_above_sensors,
				tid,
				safe_height_from_ground
			);
		else
			carmen_prob_models_update_log_odds_of_cells_hit_by_rays(map_set->log_odds_snapshot_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors, tid, safe_height_from_ground);

		if (update_and_merge_with_mapper_saved_maps && use_remission)
			carmen_prob_models_update_intensity_of_cells_hit_by_rays(
				map_set->sum_remission_snapshot_map,
				map_set->sum_sqr_remission_snapshot_map,
				map_set->count_remission_snapshot_map,
				sensor_params,
				sensor_data,
				highest_sensor,
				safe_range_above_sensors,
				NULL,
				tid,
				safe_height_from_ground
			);
	}
}


void
update_log_odds_of_cells_in_the_velodyne_perceptual_field(carmen_map_set_t *map_set, sensor_parameters_t *sensor_params,
		sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global, int point_cloud_index, int update_cells_crossed_by_rays,
		int build_snapshot_map __attribute__ ((unused)))
{
	int tid = omp_get_thread_num();
	spherical_point_cloud v_zt = sensor_data->points[point_cloud_index];
	int N = v_zt.num_points / sensor_params->vertical_resolution;

	double v = sensor_data->robot_velocity[point_cloud_index].x;
	double phi = sensor_data->robot_phi[point_cloud_index];

	double dt = sensor_params->time_spent_by_each_scan;
	double dt1 = sensor_data->points_timestamp[point_cloud_index] - sensor_data->robot_timestamp[point_cloud_index] - (double) N * dt;
	carmen_pose_3D_t robot_interpolated_position = sensor_data->robot_pose[point_cloud_index];
	int i = 0;

	//	plot_data = fopen("plot_data.dat", "w");
	// Ray-trace the grid
	carmen_pose_3D_t robot_pose = sensor_data->robot_pose[point_cloud_index];
	robot_pose.position.z = 0.0;
	robot_pose.orientation.pitch = 0.0;
	robot_pose.orientation.roll = 0.0;

	initialize_virtual_scan_message_update(VELODYNE, robot_pose, robot_interpolated_position, dt * (double) N, v_zt.sphere_points[sensor_params->vertical_resolution * (N - 1)].horizontal_angle,
			v, phi, N, point_cloud_index, r_matrix_robot_to_global, sensor_params, sensor_data);
	for (int j = 0; j < N; j += 1)
	{
		i = j * sensor_params->vertical_resolution;
		double dt2 = j * dt;
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(robot_pose,
				dt1 + dt2, v, phi, car_config.distance_between_front_and_rear_axles);
		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

		change_sensor_rear_range_max(sensor_params, v_zt.sphere_points[i].horizontal_angle);

		carmen_prob_models_compute_relevant_map_coordinates_with_remission_check(sensor_data, sensor_params, i, robot_interpolated_position.position,
				sensor_params->sensor_support_pose, r_matrix_robot_to_global, sensor_params->support_to_car_matrix,
				robot_wheel_radius, map_set->x_origin, map_set->y_origin, &car_config, robot_near_strong_slow_down_annotation, tid, use_remission);

		if (use_neural_mapper)
			neural_mapper_update_input_maps(sensor_data, sensor_params, tid, map_set->log_odds_snapshot_map, map_config, map_set->x_origin, map_set->y_origin, highest_sensor, safe_range_above_sensors);

//		fprintf(plot_data, "%lf %lf %lf",
//				sensor_data->ray_origin_in_the_floor[tid][1].x,
//				sensor_data->ray_origin_in_the_floor[tid][1].y,
//				sensor_data->ray_size_in_the_floor[tid][1]);
//		fprintf(plot_data, "%lf %lf %d",
//				sensor_data->ray_position_in_the_floor[tid][0].x,
//				sensor_data->ray_position_in_the_floor[tid][0].y, 1);
		carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(sensor_data, sensor_params, i, highest_sensor, safe_range_above_sensors,
				robot_near_strong_slow_down_annotation, tid);

//		build_virtual_scan_message_velodyne(tid, sensor_params, sensor_data, log_odds_snapshot_map);

		if (update_cells_crossed_by_rays == UPDATE_CELLS_CROSSED_BY_RAYS)
		{
			if (create_map_sum_and_count)
				carmen_prob_models_update_sum_and_count_cells_crossed_by_ray(map_set->occupancy_map, map_set->sum_occupancy_map, map_set->count_occupancy_map, sensor_params, sensor_data, tid);
			else
				carmen_prob_models_update_cells_crossed_by_ray(map_set->occupancy_map, sensor_params, sensor_data, tid);
		}
		// carmen_prob_models_upgrade_log_odds_of_cells_hit_by_rays(map, sensor_params, sensor_data);
		if (create_map_sum_and_count)
			carmen_prob_models_update_sum_and_count_of_cells_hit_by_rays_into_log_odds_snapshot_map(map_set->log_odds_snapshot_map, map_set->sum_occupancy_map, map_set->count_occupancy_map,
					sensor_params, sensor_data, highest_sensor, safe_range_above_sensors, tid, safe_height_from_ground);
		else
			carmen_prob_models_update_log_odds_of_cells_hit_by_rays(map_set->log_odds_snapshot_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors, tid, safe_height_from_ground);

		if (update_and_merge_with_mapper_saved_maps && use_remission)
			carmen_prob_models_update_intensity_of_cells_hit_by_rays(map_set->sum_remission_map, map_set->sum_sqr_remission_map, map_set->count_remission_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors, NULL, tid, safe_height_from_ground);

		carmen_prob_models_force_update_log_odds_of_cells_hit_by_rays(map_set->log_odds_snapshot_map, sensor_params, sensor_data, min_force_obstacle_height, max_force_obstacle_height, tid);

		//Lucas: Mapa para deteccao de objetos moveis
		//carmen_prob_models_update_log_odds_of_cells_hit_by_rays(&moving_objects_raw_map, sensor_params, sensor_data, highest_sensor, safe_range_above_sensors);
		//build_front_laser_message_from_velodyne_point_cloud (sensor_params, sensor_data, v_zt, i);
		//i = i +  sensor_params->vertical_resolution;
//		fprintf(plot_data, "\n");
	}
//	fprintf(plot_data, "\n");
//	fclose(plot_data);
//	system("cp plot_data.dat plot_data2.dat");
}


double
get_acceleration(double v, double timestamp)
{
	static double a = 0.0;
	static double previous_v = 0.0;
	static double previous_timestamp = 0.0;

	if (previous_timestamp == 0.0)
	{
		previous_timestamp = timestamp;
		return (0.0);
	}

	double dt = timestamp - previous_timestamp;
	if (dt < 0.01)
		return (a);

	double current_a = (v - previous_v) / dt;

	a = a + 0.5 * (current_a - a); // para suaviar um pouco

	previous_v = v;
	previous_timestamp = timestamp;

	return (a);
}

//Tem outra variavel com mesmo nome na funcao acima
//FILE *plot_data;

static void
update_log_odds_of_cells_in_the_laser_ldmrs_perceptual_field(carmen_map_set_t *map_set, sensor_parameters_t *sensor_params,
		sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global, int point_cloud_index,
		int update_cells_crossed_by_rays __attribute__ ((unused)), int build_snapshot_map __attribute__ ((unused)))
{
	int tid = omp_get_thread_num();
	spherical_point_cloud v_zt = sensor_data->points[point_cloud_index];
	int N = v_zt.num_points / sensor_params->vertical_resolution;

	double v = sensor_data->robot_velocity[point_cloud_index].x;
	double phi = sensor_data->robot_phi[point_cloud_index];

	// O codigo abaixo tenta nao registrar o LDMRS no mapa quando de freiadas
	static double blind_timestamp = 0.0;
	double a = get_acceleration(v, sensor_data->robot_timestamp[point_cloud_index]);
	if (a < -sensor_params->cutoff_negative_acceleration)
	{
		blind_timestamp = sensor_data->points_timestamp[point_cloud_index];
		return;
	}
	if ((sensor_data->points_timestamp[point_cloud_index] - blind_timestamp) < 0.5)
		return;

	double dt = sensor_params->time_spent_by_each_scan;
	double dt1 = sensor_data->points_timestamp[point_cloud_index] - sensor_data->robot_timestamp[point_cloud_index] - (double) N * dt;
	carmen_pose_3D_t robot_interpolated_position = sensor_data->robot_pose[point_cloud_index];
	int i = 0;

//	plot_data = fopen("plot_data.dat", "a");
	carmen_pose_3D_t robot_pose = sensor_data->robot_pose[point_cloud_index];
	robot_pose.position.z = 0.0;
	robot_pose.orientation.pitch = 0.0;
	robot_pose.orientation.roll = 0.0;

	initialize_virtual_scan_message_update(LASER_LDMRS, robot_pose, robot_interpolated_position, dt * (double) N, v_zt.sphere_points[sensor_params->vertical_resolution * (N - 1)].horizontal_angle,
			v, phi, N, point_cloud_index, r_matrix_robot_to_global, sensor_params, sensor_data);
	for (int j = 0; j < N; j += 1)
	{
		i = j * sensor_params->vertical_resolution;
		double dt2 = j * dt;
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(robot_pose,
				dt1 + dt2, v, phi, car_config.distance_between_front_and_rear_axles);
		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

		carmen_prob_models_compute_relevant_map_coordinates_with_remission_check(sensor_data, sensor_params, i, robot_interpolated_position.position,
				sensor_params->sensor_support_pose, r_matrix_robot_to_global, sensor_params->support_to_car_matrix,
				robot_wheel_radius, map_set->x_origin, map_set->y_origin, &car_config, robot_near_strong_slow_down_annotation, tid, 0);

//		fprintf(plot_data, "%lf %lf %lf",
//				sensor_data->ray_position_in_the_floor[tid][1].x,
//				sensor_data->ray_position_in_the_floor[tid][1].y,
//				sensor_data->ray_size_in_the_floor[tid][1]);
//		fprintf(plot_data, "%lf %lf %d",
//				sensor_data->ray_position_in_the_floor[tid][0].x,
//				sensor_data->ray_position_in_the_floor[tid][0].y, 1);
//		carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(sensor_data, sensor_params, i, highest_sensor, safe_range_above_sensors,
//				robot_near_bump_or_barrier, tid);
//
//		if (update_cells_crossed_by_rays == UPDATE_CELLS_CROSSED_BY_RAYS)
//			carmen_prob_models_update_cells_crossed_by_ray(snapshot_map, sensor_params, sensor_data, tid);

		build_virtual_scan_message_ldmrs(tid, sensor_params, sensor_data, map_set);

		carmen_prob_models_update_log_odds_of_cells_hit_by_ldmrs_rays(map_set->log_odds_snapshot_map, sensor_params, sensor_data, tid);

//		fprintf(plot_data, "\n");
	}
//	fprintf(plot_data, "dt %lf, dt1 %lf\n", dt, dt1);
//	fclose(plot_data);
//	system("cp plot_data.dat plot_data2.dat");
}


void
set_map_equal_offline_map(carmen_map_t *current_map, carmen_map_set_t *map_set)
{
	int xi, yi;

	for (xi = 0; xi < current_map->config.x_size; xi++)
		for (yi = 0; yi < current_map->config.y_size; yi++)
			current_map->map[xi][yi] = map_set->offline_map->map[xi][yi];
}


void
add_offline_map_over_unknown(carmen_map_t *current_map, carmen_map_set_t *map_set)
{
	for (int xi = 0; xi < current_map->config.x_size; xi++)
		for (int yi = 0; yi < current_map->config.y_size; yi++)
			if (current_map->map[xi][yi] < 0.0)
				current_map->map[xi][yi] = map_set->offline_map->map[xi][yi];
}


void
map_decay_to_offline_map(carmen_map_set_t *map_set)
{
//	#pragma omp for
	for (int i = 0; i < map_set->occupancy_map->config.x_size * map_set->occupancy_map->config.y_size; i++)
	{
		if (map_set->occupancy_map->complete_map[i] >= 0.0)
		{
			//map_set->occupancy_map->complete_map[i] = (50.0 * map_set->occupancy_map->complete_map[i] + offline_map.complete_map[i]) / 51.0;
			if (use_unity_simulator)
				map_set->occupancy_map->complete_map[i] = map_set->offline_map->complete_map[i];
			else
				map_set->occupancy_map->complete_map[i] = (3.0 * map_set->occupancy_map->complete_map[i] + map_set->offline_map->complete_map[i]) / 4.0;
		}
		else
			map_set->occupancy_map->complete_map[i] = map_set->offline_map->complete_map[i];
	}
}


void
clear_log_odds_map(carmen_map_t *log_odds_snapshot_map, double log_odds_l0)
{
	for (int i = 0; i < log_odds_snapshot_map->config.x_size * log_odds_snapshot_map->config.y_size; i++)
		log_odds_snapshot_map->complete_map[i] = log_odds_l0;
}


int
run_mapper_with_remision_threshold(carmen_map_set_t *map_set, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data,
		rotation_matrix *r_matrix_robot_to_global, double rays_threshold_to_merge_between_maps)
{
	static int first = 1;

	if (!globalpos_initialized)
		return (0);

	if (first)
	{
		map_set->log_odds_snapshot_map = (carmen_map_t *) calloc(1, sizeof(carmen_map_t));
		first = 0;
	}

//#pragma omp parallel num_threads(number_of_threads)
	{
		map_set->log_odds_snapshot_map = carmen_prob_models_check_if_new_log_odds_snapshot_map_allocation_is_needed(map_set->log_odds_snapshot_map, map_set->occupancy_map);

		if (sensor_params->sensor_type == LASER_LDMRS)
		{
			update_log_odds_of_cells_in_the_laser_ldmrs_perceptual_field(map_set,
				sensor_params,
				sensor_data,
				r_matrix_robot_to_global,
				sensor_data->point_cloud_index,
				UPDATE_CELLS_CROSSED_BY_RAYS,
				update_and_merge_with_snapshot_map);

			carmen_prob_models_clear_cells_hit_by_single_ray(map_set->log_odds_snapshot_map,
				sensor_params->log_odds.log_odds_occ,
				sensor_params->log_odds.log_odds_l0);

			carmen_prob_models_overwrite_current_map_with_log_odds_snapshot_map_and_clear_snapshot_map(map_set->occupancy_map, map_set->log_odds_snapshot_map,
				sensor_params->log_odds.log_odds_l0);
		}
		else // Velodyne and others
		{
//			if (decay_to_offline_map)
//				map_decay_to_offline_map(map_set);

			carmen_pose_3D_t neural_mapper_robot_pose = sensor_data->robot_pose[sensor_data->point_cloud_index];

			update_log_odds_of_cells_in_the_velodyne_perceptual_field_with_snapshot_maps(map_set,
				sensor_params,
				sensor_data,
				r_matrix_robot_to_global,
				UPDATE_CELLS_CROSSED_BY_RAYS
			);

			carmen_prob_models_update_current_map_with_log_odds_snapshot_map_and_clear_snapshot_map(map_set->snapshot_map, map_set->log_odds_snapshot_map,
				sensor_params->log_odds.log_odds_l0
			);

			if (use_neural_mapper)
			{
				if (generate_neural_mapper_dataset)
				{
					bool get_next_map = neural_mapper_compute_travelled_distance(
						&neural_mapper_car_position_according_to_map,
						neural_mapper_robot_pose,
						map_set->x_origin,
						map_set->y_origin,
						neural_mapper_data_pace
					);

					neural_mapper_update_output_map(*(map_set->offline_map), neural_mapper_car_position_according_to_map);
					//					neural_mapper_export_dataset_as_png(get_next_map, neural_mapper_dataset_path);
										neural_mapper_export_dataset_as_binary_file(get_next_map, neural_mapper_dataset_path,
																	sensor_data->current_timestamp, neural_mapper_robot_pose);
				}
				neural_mapper_update_queue_and_clear_maps();
			}
		}
	}

	mapper_merge_snapshot_map(map_set->count_remission_map, map_set->snapshot_map, map_set->occupancy_map, rays_threshold_to_merge_between_maps);
	mapper_merge_snapshot_map(map_set->count_remission_map, map_set->sum_remission_snapshot_map, map_set->sum_remission_map, rays_threshold_to_merge_between_maps);
	mapper_merge_snapshot_map(map_set->count_remission_map, map_set->sum_sqr_remission_snapshot_map, map_set->sum_sqr_remission_map, rays_threshold_to_merge_between_maps);
	mapper_merge_snapshot_map(map_set->count_remission_map, map_set->count_remission_snapshot_map, map_set->count_remission_map, rays_threshold_to_merge_between_maps);

	return (1);
}


int
run_mapper(carmen_map_set_t *map_set, sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, rotation_matrix *r_matrix_robot_to_global)
{
	//int N = 4;
	static int first = 1;

	if (!globalpos_initialized)
		return (0);

	if (first)
	{
		map_set->log_odds_snapshot_map = (carmen_map_t *) calloc(1, sizeof(carmen_map_t));
		first = 0;
	}

//#pragma omp parallel num_threads(number_of_threads)
	{
		map_set->log_odds_snapshot_map = carmen_prob_models_check_if_new_log_odds_snapshot_map_allocation_is_needed(map_set->log_odds_snapshot_map, map_set->occupancy_map);
		//set_map_equal_offline_map(&map, map_set);
		//add_offline_map_over_unknown(&map, map_set);

		if (sensor_params->sensor_type == LASER_LDMRS)
		{
			update_log_odds_of_cells_in_the_laser_ldmrs_perceptual_field(map_set, sensor_params, sensor_data, r_matrix_robot_to_global, sensor_data->point_cloud_index, UPDATE_CELLS_CROSSED_BY_RAYS, update_and_merge_with_snapshot_map);
			carmen_prob_models_clear_cells_hit_by_single_ray(map_set->log_odds_snapshot_map, sensor_params->log_odds.log_odds_occ,
					sensor_params->log_odds.log_odds_l0);
			carmen_prob_models_overwrite_current_map_with_log_odds_snapshot_map_and_clear_snapshot_map(map_set->occupancy_map, map_set->log_odds_snapshot_map,
					sensor_params->log_odds.log_odds_l0);
		}
		else // Velodyne and others
		{
//			if (decay_to_offline_map)
//				map_decay_to_offline_map(map_set);

			carmen_pose_3D_t neural_mapper_robot_pose = sensor_data->robot_pose[sensor_data->point_cloud_index];

			// @@@ Alberto: Mapa padrao Lucas -> colocar DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS ao inves de UPDATE_CELLS_CROSSED_BY_RAYS
			update_log_odds_of_cells_in_the_velodyne_perceptual_field(map_set, sensor_params, sensor_data, r_matrix_robot_to_global,
					sensor_data->point_cloud_index, UPDATE_CELLS_CROSSED_BY_RAYS, update_and_merge_with_snapshot_map);

			if (!generate_neural_mapper_dataset && use_neural_mapper)
			{
				if (map_set->offline_map->complete_map != NULL)
				{
					int size_trained = neural_mapper_max_distance_meters / map_set->log_odds_snapshot_map->config.resolution * 2;
					clear_log_odds_map(map_set->log_odds_snapshot_map, sensor_params->log_odds.log_odds_l0);
					neural_map_run_foward(map_set->log_odds_snapshot_map, size_trained, &neural_mapper_robot_pose, map_set->x_origin, map_set->y_origin);
					carmen_prob_models_update_current_map_with_log_odds_snapshot_map_and_clear_snapshot_map(map_set->occupancy_map, map_set->log_odds_snapshot_map,
																										sensor_params->log_odds.log_odds_l0);
					// carmen_grid_mapping_save_map((char *) "neural_map_teste.map", &map);
				}
			}
			else
				carmen_prob_models_update_current_map_with_log_odds_snapshot_map_and_clear_snapshot_map(map_set->occupancy_map, map_set->log_odds_snapshot_map,
						sensor_params->log_odds.log_odds_l0);
			// carmen_grid_mapping_save_map((char *) "test.map", &map);

			if (use_neural_mapper)//To generate the dataset to use in Neural Mapper training.
			{
				if (generate_neural_mapper_dataset)
				{
					bool get_next_map = neural_mapper_compute_travelled_distance(&neural_mapper_car_position_according_to_map, neural_mapper_robot_pose,
							map_set->x_origin, map_set->y_origin, neural_mapper_data_pace);

					neural_mapper_update_output_map(*(map_set->offline_map), neural_mapper_car_position_according_to_map);
					// neural_mapper_export_dataset_as_png(get_next_map, neural_mapper_dataset_path);
					neural_mapper_export_dataset_as_binary_file(get_next_map, neural_mapper_dataset_path, sensor_data->current_timestamp, neural_mapper_robot_pose);
				}
				neural_mapper_update_queue_and_clear_maps();
			}
		}
	}

	return (1);
}


void
initialize_first_map_block_origin(char *map_path, carmen_map_set_t *map_set, carmen_map_t *current_carmen_map, carmen_position_t *map_origin, char map_type)
{
	current_carmen_map->complete_map = NULL;
	map_set->x_origin = map_origin->x;
	map_set->y_origin = map_origin->y;

	if (update_and_merge_with_mapper_saved_maps)
	{
		carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, map_type, map_set->x_origin, map_set->y_origin, current_carmen_map);

		if (current_carmen_map->complete_map == NULL)
			carmen_grid_mapping_initialize_map(current_carmen_map, current_carmen_map->config.x_size, current_carmen_map->config.resolution, map_type);
	}
	else
		carmen_grid_mapping_get_buffered_map(map_set, map_set->x_origin, map_set->y_origin, current_carmen_map, map_type);
}


int
mapper_change_map_origin_to_another_map_block_with_clones(char *map_path, carmen_map_set_t *map_set,
		carmen_position_t *map_origin, bool save_map, bool force_saving_new_map)
{
	static int first_time = 1;

	if (first_time)
	{
		initialize_first_map_block_origin(map_path, map_set, map_set->occupancy_map, map_origin, 'm');

		if (publish_moving_objects_raw_map)
			initialize_first_map_block_origin(map_path, map_set, map_set->moving_objects_raw_map, map_origin, 'm');

		if (create_map_sum_and_count)
		{
			initialize_first_map_block_origin(map_path, map_set, map_set->sum_occupancy_map, map_origin, 'u');
			initialize_first_map_block_origin(map_path, map_set, map_set->count_occupancy_map, map_origin, 'o');
		}

		if (use_remission)
		{
			initialize_first_map_block_origin(map_path, map_set, map_set->sum_remission_map, map_origin, 's');
			initialize_first_map_block_origin(map_path, map_set, map_set->sum_sqr_remission_map, map_origin, '2');
			initialize_first_map_block_origin(map_path, map_set, map_set->count_remission_map, map_origin, 'c');
		}

		carmen_grid_mapping_create_new_map(map_set->new_occupancy_map, map_set->occupancy_map->config.x_size, map_set->occupancy_map->config.y_size, map_set->occupancy_map->config.resolution, 'm');

		if (create_map_sum_and_count)
		{
			carmen_grid_mapping_create_new_map(map_set->new_sum_occupancy_map, map_set->sum_occupancy_map->config.x_size, map_set->sum_occupancy_map->config.y_size, map_set->sum_occupancy_map->config.resolution, 'u');
			carmen_grid_mapping_create_new_map(map_set->new_count_occupancy_map, map_set->count_occupancy_map->config.x_size, map_set->count_occupancy_map->config.y_size, map_set->count_occupancy_map->config.resolution, 'o');
		}

		if (use_remission)
		{
			carmen_grid_mapping_create_new_map(map_set->new_sum_remission_map, map_set->sum_remission_map->config.x_size, map_set->sum_remission_map->config.y_size, map_set->sum_remission_map->config.resolution, 's');
			carmen_grid_mapping_create_new_map(map_set->new_sum_sqr_remission_map, map_set->sum_sqr_remission_map->config.x_size, map_set->sum_sqr_remission_map->config.y_size, map_set->sum_sqr_remission_map->config.resolution, '2');
			carmen_grid_mapping_create_new_map(map_set->new_count_remission_map, map_set->count_remission_map->config.x_size, map_set->count_remission_map->config.y_size, map_set->count_remission_map->config.resolution, 'c');
		}

		first_time = 0;
	}

	int map_origin_changed = carmen_grid_mapping_is_map_changed(map_origin, map_set->x_origin, map_set->y_origin);
	if (force_saving_new_map || map_origin_changed)
	{
		map_set->x_origin = map_origin->x;
		map_set->y_origin = map_origin->y;

		if (update_and_merge_with_mapper_saved_maps)
		{
			if (save_map && offline_map_available && (map_set->occupancy_map->complete_map != NULL) && (map_set->occupancy_map->config.x_origin != 0.0))
			{
				carmen_grid_mapping_save_block_map_by_origin(map_path, 'm', map_set->occupancy_map);

				if (create_map_sum_and_count)
				{
					carmen_grid_mapping_save_block_map_by_origin(map_path, 'u', map_set->sum_occupancy_map);
					carmen_grid_mapping_save_block_map_by_origin(map_path, 'o', map_set->count_occupancy_map);
				}

				if (use_remission)
				{
					carmen_grid_mapping_save_block_map_by_origin(map_path, 's', map_set->sum_remission_map);
					carmen_grid_mapping_save_block_map_by_origin(map_path, '2', map_set->sum_sqr_remission_map);
					carmen_grid_mapping_save_block_map_by_origin(map_path, 'c', map_set->count_remission_map);
				}
			}

			// get new map with integrated information of the old map
			carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'm', map_set->x_origin, map_set->y_origin, map_set->new_occupancy_map);

			if (create_map_sum_and_count)
			{
				carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'u', map_set->x_origin, map_set->y_origin, map_set->new_sum_occupancy_map);
				carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'o', map_set->x_origin, map_set->y_origin, map_set->new_count_occupancy_map);
			}

			if (use_remission)
			{
				carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 's', map_set->x_origin, map_set->y_origin, map_set->new_sum_remission_map);
				carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, '2', map_set->x_origin, map_set->y_origin, map_set->new_sum_sqr_remission_map);
				carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'c', map_set->x_origin, map_set->y_origin, map_set->new_count_remission_map);
			}
		}

		carmen_grid_mapping_update_map_buffer(map_set, map_set->occupancy_map, 'm');
		carmen_grid_mapping_get_buffered_map(map_set, map_set->x_origin, map_set->y_origin, map_set->new_occupancy_map, 'm');
		carmen_grid_mapping_swap_maps_and_clear_old_map(map_set->occupancy_map, map_set->new_occupancy_map);

		if (create_map_sum_and_count)
		{
			carmen_grid_mapping_update_map_buffer(map_set, map_set->sum_occupancy_map, 'u');
			carmen_grid_mapping_get_buffered_map(map_set, map_set->x_origin, map_set->y_origin, map_set->new_sum_occupancy_map, 'u');
			carmen_grid_mapping_swap_maps_and_clear_old_map(map_set->sum_occupancy_map, map_set->new_sum_occupancy_map);

			carmen_grid_mapping_update_map_buffer(map_set, map_set->count_occupancy_map, 'o');
			carmen_grid_mapping_get_buffered_map(map_set, map_set->x_origin, map_set->y_origin, map_set->new_count_occupancy_map, 'o');
			carmen_grid_mapping_swap_maps_and_clear_old_map(map_set->count_occupancy_map, map_set->new_count_occupancy_map);
		}

		if (use_remission)
		{
			carmen_grid_mapping_update_map_buffer(map_set, map_set->sum_remission_map, 's');
			carmen_grid_mapping_get_buffered_map(map_set, map_set->x_origin, map_set->y_origin, map_set->new_sum_remission_map, 's');
			carmen_grid_mapping_swap_maps_and_clear_old_map(map_set->sum_remission_map, map_set->new_sum_remission_map);

			carmen_grid_mapping_update_map_buffer(map_set, map_set->sum_sqr_remission_map, '2');
			carmen_grid_mapping_get_buffered_map(map_set, map_set->x_origin, map_set->y_origin, map_set->new_sum_sqr_remission_map, '2');
			carmen_grid_mapping_swap_maps_and_clear_old_map(map_set->sum_sqr_remission_map, map_set->new_sum_sqr_remission_map);

			carmen_grid_mapping_update_map_buffer(map_set, map_set->count_remission_map, 'c');
			carmen_grid_mapping_get_buffered_map(map_set, map_set->x_origin, map_set->y_origin, map_set->new_count_remission_map, 'c');
			carmen_grid_mapping_swap_maps_and_clear_old_map(map_set->count_remission_map, map_set->new_count_remission_map);
		}
	}

	map_set->occupancy_map->config.x_origin = map_set->snapshot_map->config.x_origin = map_set->x_origin;
	map_set->occupancy_map->config.y_origin = map_set->snapshot_map->config.y_origin = map_set->y_origin;
	mapper_clone_map(map_set->occupancy_map, map_set->snapshot_map, 'm');

	if (publish_moving_objects_raw_map)
	{
		map_set->moving_objects_raw_map->config.x_origin = map_set->x_origin;
		map_set->moving_objects_raw_map->config.y_origin = map_set->y_origin;
	}

	if (create_map_sum_and_count)
	{
		map_set->sum_occupancy_map->config.x_origin = map_set->x_origin;
		map_set->sum_occupancy_map->config.y_origin = map_set->y_origin;

		map_set->count_occupancy_map->config.x_origin = map_set->x_origin;
		map_set->count_occupancy_map->config.y_origin = map_set->y_origin;
	}

	if (use_remission)
	{
		map_set->sum_remission_map->config.x_origin = map_set->x_origin;
		map_set->sum_remission_map->config.y_origin = map_set->y_origin;

		map_set->sum_sqr_remission_map->config.x_origin = map_set->x_origin;
		map_set->sum_sqr_remission_map->config.y_origin = map_set->y_origin;

		map_set->count_remission_snapshot_map->config.x_origin = map_set->sum_sqr_remission_snapshot_map->config.x_origin = map_set->x_origin;
		map_set->count_remission_snapshot_map->config.y_origin = map_set->sum_sqr_remission_snapshot_map->config.y_origin = map_set->y_origin;

		mapper_clone_map(map_set->sum_remission_map, map_set->sum_remission_snapshot_map, 's');
		mapper_clone_map(map_set->sum_sqr_remission_map, map_set->sum_sqr_remission_snapshot_map, '2');
		mapper_clone_map(map_set->count_remission_map, map_set->count_remission_snapshot_map, 'c');
	}

	return (map_origin_changed);
}


bool
map_valid(carmen_map_t *map)
{
	if (
			(map->map[map->config.x_size / 2][0] > 1.0) ||
			(map->map[0][map->config.y_size / 2] > 1.0) ||
			(map->map[map->config.x_size - 1][map->config.y_size / 2] > 1.0) ||
			(map->map[map->config.x_size / 2][map->config.y_size - 1] > 1.0)
	   )
		return (false);
	else
		return (true);
}


int
mapper_change_map_origin_to_another_map_block(char *map_path, carmen_map_set_t *map_set,
		carmen_position_t *map_origin, bool save_map, bool force_saving_new_map)
{
	static int first_time = 1;

	if (first_time)
	{
		initialize_first_map_block_origin(map_path, map_set, map_set->occupancy_map, map_origin, 'm');

		if (publish_moving_objects_raw_map)
			initialize_first_map_block_origin(map_path, map_set, map_set->moving_objects_raw_map, map_origin, 'm');

		if (create_map_sum_and_count)
		{
			initialize_first_map_block_origin(map_path, map_set, map_set->sum_occupancy_map, map_origin, 'u');
			initialize_first_map_block_origin(map_path, map_set, map_set->count_occupancy_map, map_origin, 'o');
		}

		if (use_remission)
		{
			initialize_first_map_block_origin(map_path, map_set, map_set->sum_remission_map, map_origin, 's');
			initialize_first_map_block_origin(map_path, map_set, map_set->sum_sqr_remission_map, map_origin, '2');
			initialize_first_map_block_origin(map_path, map_set, map_set->count_remission_map, map_origin, 'c');
		}

		carmen_grid_mapping_create_new_map(map_set->new_occupancy_map, map_set->occupancy_map->config.x_size, map_set->occupancy_map->config.y_size, map_set->occupancy_map->config.resolution, 'm');

		if (create_map_sum_and_count)
		{
			carmen_grid_mapping_create_new_map(map_set->new_sum_occupancy_map, map_set->sum_occupancy_map->config.x_size, map_set->sum_occupancy_map->config.y_size, map_set->sum_occupancy_map->config.resolution, 'u');
			carmen_grid_mapping_create_new_map(map_set->new_count_occupancy_map, map_set->count_occupancy_map->config.x_size, map_set->count_occupancy_map->config.y_size, map_set->count_occupancy_map->config.resolution, 'o');
		}

		if (use_remission)
		{
			carmen_grid_mapping_create_new_map(map_set->new_sum_remission_map, map_set->sum_remission_map->config.x_size, map_set->sum_remission_map->config.y_size, map_set->sum_remission_map->config.resolution, 's');
			carmen_grid_mapping_create_new_map(map_set->new_sum_sqr_remission_map, map_set->sum_sqr_remission_map->config.x_size, map_set->sum_sqr_remission_map->config.y_size, map_set->sum_sqr_remission_map->config.resolution, '2');
			carmen_grid_mapping_create_new_map(map_set->new_count_remission_map, map_set->count_remission_map->config.x_size, map_set->count_remission_map->config.y_size, map_set->count_remission_map->config.resolution, 'c');
		}

		first_time = 0;
	}

	int map_origin_changed = carmen_grid_mapping_is_map_changed(map_origin, map_set->x_origin, map_set->y_origin);
	if (force_saving_new_map || map_origin_changed)
	{
		map_set->x_origin = map_origin->x;
		map_set->y_origin = map_origin->y;

		if (update_and_merge_with_mapper_saved_maps)
		{
			if (save_map && offline_map_available && (map_set->occupancy_map->complete_map != NULL) && (map_set->occupancy_map->config.x_origin != 0.0))
			{
				carmen_grid_mapping_save_block_map_by_origin(map_path, 'm', map_set->occupancy_map);

				if (create_map_sum_and_count)
				{
					carmen_grid_mapping_save_block_map_by_origin(map_path, 'u', map_set->sum_occupancy_map);
					carmen_grid_mapping_save_block_map_by_origin(map_path, 'o', map_set->count_occupancy_map);
				}

				if (use_remission)
				{
					carmen_grid_mapping_save_block_map_by_origin(map_path, 's', map_set->sum_remission_map);
					carmen_grid_mapping_save_block_map_by_origin(map_path, '2', map_set->sum_sqr_remission_map);
					carmen_grid_mapping_save_block_map_by_origin(map_path, 'c', map_set->count_remission_map);
				}
			}

			// get new map with integrated information of the old map
			carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'm', map_set->x_origin, map_set->y_origin, map_set->new_occupancy_map);

			if (create_map_sum_and_count)
			{
				carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'u', map_set->x_origin, map_set->y_origin, map_set->new_sum_occupancy_map);
				carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'o', map_set->x_origin, map_set->y_origin, map_set->new_count_occupancy_map);
			}

			if (use_remission)
			{
				carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 's', map_set->x_origin, map_set->y_origin, map_set->new_sum_remission_map);
				carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, '2', map_set->x_origin, map_set->y_origin, map_set->new_sum_sqr_remission_map);
				carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, 'c', map_set->x_origin, map_set->y_origin, map_set->new_count_remission_map);
			}
		}

		carmen_grid_mapping_update_map_buffer(map_set, map_set->occupancy_map, 'm');
		carmen_grid_mapping_get_buffered_map(map_set, map_set->x_origin, map_set->y_origin, map_set->new_occupancy_map, 'm');
		carmen_grid_mapping_swap_maps_and_clear_old_map(map_set->occupancy_map, map_set->new_occupancy_map);

		if (create_map_sum_and_count)
		{
			carmen_grid_mapping_update_map_buffer(map_set, map_set->sum_occupancy_map, 'u');
			carmen_grid_mapping_get_buffered_map(map_set, map_set->x_origin, map_set->y_origin, map_set->new_sum_occupancy_map, 'u');
			carmen_grid_mapping_swap_maps_and_clear_old_map(map_set->sum_occupancy_map, map_set->new_sum_occupancy_map);

			carmen_grid_mapping_update_map_buffer(map_set, map_set->count_occupancy_map, 'o');
			carmen_grid_mapping_get_buffered_map(map_set, map_set->x_origin, map_set->y_origin, map_set->new_count_occupancy_map, 'o');
			carmen_grid_mapping_swap_maps_and_clear_old_map(map_set->count_occupancy_map, map_set->new_count_occupancy_map);
		}

		if (use_remission)
		{
			carmen_grid_mapping_update_map_buffer(map_set, map_set->sum_remission_map, 's');
			carmen_grid_mapping_get_buffered_map(map_set, map_set->x_origin, map_set->y_origin, map_set->new_sum_remission_map, 's');
			carmen_grid_mapping_swap_maps_and_clear_old_map(map_set->sum_remission_map, map_set->new_sum_remission_map);

			carmen_grid_mapping_update_map_buffer(map_set, map_set->sum_sqr_remission_map, '2');
			carmen_grid_mapping_get_buffered_map(map_set, map_set->x_origin, map_set->y_origin, map_set->new_sum_sqr_remission_map, '2');
			carmen_grid_mapping_swap_maps_and_clear_old_map(map_set->sum_sqr_remission_map, map_set->new_sum_sqr_remission_map);

			carmen_grid_mapping_update_map_buffer(map_set, map_set->count_remission_map, 'c');
			carmen_grid_mapping_get_buffered_map(map_set, map_set->x_origin, map_set->y_origin, map_set->new_count_remission_map, 'c');
			carmen_grid_mapping_swap_maps_and_clear_old_map(map_set->count_remission_map, map_set->new_count_remission_map);
		}
	}

	map_set->occupancy_map->config.x_origin = map_set->x_origin;
	map_set->occupancy_map->config.y_origin = map_set->y_origin;

	if (publish_moving_objects_raw_map)
	{
		map_set->moving_objects_raw_map->config.x_origin = map_set->x_origin;
		map_set->moving_objects_raw_map->config.y_origin = map_set->y_origin;
	}

	if (create_map_sum_and_count)
	{
		map_set->sum_occupancy_map->config.x_origin = map_set->x_origin;
		map_set->sum_occupancy_map->config.y_origin = map_set->y_origin;

		map_set->count_occupancy_map->config.x_origin = map_set->x_origin;
		map_set->count_occupancy_map->config.y_origin = map_set->y_origin;
	}

	if (use_remission)
	{
		map_set->sum_remission_map->config.x_origin = map_set->x_origin;
		map_set->sum_remission_map->config.y_origin = map_set->y_origin;

		map_set->sum_sqr_remission_map->config.x_origin = map_set->x_origin;
		map_set->sum_sqr_remission_map->config.y_origin = map_set->y_origin;

		map_set->count_remission_map->config.x_origin = map_set->x_origin;
		map_set->count_remission_map->config.y_origin = map_set->y_origin;
	}

	return (map_origin_changed);
}


int
run_snapshot_mapper(carmen_map_set_t *map_set)
{	// Esta funcao nao esta funcionando direito pois mistura mapas log odds com mapas probabilisticos...
	int i;
	int current_point_cloud_index;//, before_point_cloud_index;
	static rotation_matrix *r_matrix_robot_to_global = NULL;

	map_set->snapshot_map = carmen_prob_models_check_if_new_snapshot_map_allocation_is_needed(map_set->snapshot_map, map_set->occupancy_map);

	if (!globalpos_initialized)
		return (0);

	if (sensors_params[0].alive)
	{
		current_point_cloud_index =  sensors_data[0].point_cloud_index;
//		before_point_cloud_index =  ((sensors_data[0].point_cloud_index - 1) + NUM_VELODYNE_POINT_CLOUDS) % NUM_VELODYNE_POINT_CLOUDS;

//		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_robot_to_global, sensors_data[0].robot_pose[before_point_cloud_index].orientation);
//		update_cells_in_the_velodyne_perceptual_field(snapshot_map, &sensors_params[0], &sensors_data[0], r_matrix_robot_to_global, before_point_cloud_index, UPDATE_CELLS_CROSSED_BY_RAYS);
//		carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(&map, snapshot_map);

		r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_robot_to_global, sensors_data[0].robot_pose[current_point_cloud_index].orientation);
		update_log_odds_of_cells_in_the_velodyne_perceptual_field(map_set, &sensors_params[0], &sensors_data[0], r_matrix_robot_to_global, current_point_cloud_index, DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS, 0);
		carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(map_set->occupancy_map, map_set->snapshot_map);
	}

	for (i = 1; i < number_of_sensors; i++)
	{
		if (sensors_params[i].alive)
		{
			current_point_cloud_index =  sensors_data[i].point_cloud_index;
			r_matrix_robot_to_global = compute_rotation_matrix(r_matrix_robot_to_global, sensors_data[i].robot_pose[current_point_cloud_index].orientation);
			update_log_odds_of_cells_in_the_velodyne_perceptual_field(map_set, &sensors_params[i], &sensors_data[i], r_matrix_robot_to_global, current_point_cloud_index, DO_NOT_UPDATE_CELLS_CROSSED_BY_RAYS, 0);
			carmen_prob_models_overwrite_current_map_with_snapshot_map_and_clear_snapshot_map(map_set->occupancy_map, map_set->snapshot_map);
		}
	}

	return (1);
}


static void
build_sensor_point_cloud(spherical_point_cloud **points, unsigned char **intensity, int *point_cloud_index, int num_points, int max_point_buffer)
{
	(*point_cloud_index)++;
	if ((*point_cloud_index) >= max_point_buffer)
		*point_cloud_index = 0;

	if ((*points)[*point_cloud_index].num_points != num_points)
		intensity[*point_cloud_index] = (unsigned char *) realloc((void *) intensity[*point_cloud_index], num_points * sizeof(unsigned char));

	carmen_alloc_spherical_point_cloud(*points, num_points, *point_cloud_index);
}


void
build_sensor_point_cloud(spherical_point_cloud **points, unsigned char **intensity, int *point_cloud_index, int num_points, int max_point_buffer,
		int use_remission)
{
	(*point_cloud_index)++;
	if ((*point_cloud_index) >= max_point_buffer)
		*point_cloud_index = 0;

	if ((*points)[*point_cloud_index].num_points != num_points && use_remission)
		intensity[*point_cloud_index] = (unsigned char *) realloc((void *) intensity[*point_cloud_index], num_points * sizeof(unsigned char));

	carmen_alloc_spherical_point_cloud(*points, num_points, *point_cloud_index);
}


int
mapper_velodyne_partial_scan(int sensor_number, carmen_velodyne_partial_scan_message *velodyne_message)
{
	static int velodyne_message_id;
	//int ok_to_publish;

	int num_points = velodyne_message->number_of_32_laser_shots * sensors_params[sensor_number].vertical_resolution;

	ok_to_publish = 0;
	if (!globalpos_initialized)
		return (ok_to_publish);

	if (sensors_data[sensor_number].last_timestamp == 0.0)
	{
		sensors_data[sensor_number].last_timestamp = velodyne_message->timestamp;
		velodyne_message_id = -2;		// antigamente eram necessarias pelo menos 2 mensagens para ter uma volta completa de velodyne

		return (ok_to_publish);
	}

	sensors_data[sensor_number].current_timestamp = velodyne_message->timestamp;

	build_sensor_point_cloud(&sensors_data[sensor_number].points, sensors_data[sensor_number].intensity,
			&sensors_data[sensor_number].point_cloud_index, num_points, NUM_VELODYNE_POINT_CLOUDS, use_remission);

	// printf ("PARTIAL\n");
	// for (int i = 0; i < sensors_params[sensor_number].vertical_resolution; i++)
	// 	printf ("%d ", sensors_params[sensor_number].ray_order[i]);
	// printf ("\n");
	// for (int i = 0; i < sensors_params[sensor_number].vertical_resolution; i++)
	// 	printf ("%lf ", sensors_params[sensor_number].vertical_correction[i]);
	// printf ("\n");

	carmen_velodyne_partial_scan_update_points_with_remission_check(velodyne_message, sensors_params[sensor_number].vertical_resolution,
			&sensors_data[sensor_number].points[sensors_data[sensor_number].point_cloud_index],
			sensors_data[sensor_number].intensity[sensors_data[sensor_number].point_cloud_index],
			sensors_params[sensor_number].ray_order, sensors_params[sensor_number].vertical_correction, sensors_params[sensor_number].range_max,
			velodyne_message->timestamp, use_remission);

	sensors_data[sensor_number].robot_pose[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].pose;
	sensors_data[sensor_number].robot_velocity[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].velocity;
	sensors_data[sensor_number].robot_timestamp[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].timestamp;
	sensors_data[sensor_number].robot_phi[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].phi;
	sensors_data[sensor_number].points_timestamp[sensors_data[sensor_number].point_cloud_index] = velodyne_message->timestamp;

	if (velodyne_message_id >= 0)
	{
		//if (build_snapshot_map)
		ok_to_publish = 1;
		//else

		if (velodyne_message_id > 1000000)
			velodyne_message_id = 0;
	}
	velodyne_message_id++;
	sensors_data[sensor_number].last_timestamp = velodyne_message->timestamp;

	return (ok_to_publish);
}


int
update_data_params_with_lidar_data(int sensor_number, carmen_velodyne_variable_scan_message *msg)
{
	// static int msg_id;

	if (!globalpos_initialized)
		return (ok_to_publish);
	
	int num_points = msg->number_of_shots * sensors_params[sensor_number].vertical_resolution;

	// ok_to_publish = 0;

	// if (sensors_data[sensor_number].last_timestamp == 0.0)    // Todo ainda precisa????
	// {
	// 	sensors_data[sensor_number].last_timestamp = msg->timestamp;
	// 	msg_id = -2;		// antigamente eram necessarias pelo menos 2 mensagens para ter uma volta completa de velodyne

	// 	return (ok_to_publish);
	// }

	sensors_data[sensor_number].current_timestamp = msg->timestamp;

	build_sensor_point_cloud(&sensors_data[sensor_number].points, sensors_data[sensor_number].intensity,
			&sensors_data[sensor_number].point_cloud_index, num_points, NUM_VELODYNE_POINT_CLOUDS, use_remission);
	
	// printf ("VARIABLE\n");
	// for (int i = 0; i < sensors_params[sensor_number].vertical_resolution; i++)
	// 	printf ("%d ", sensors_params[sensor_number].ray_order[i]);
	// printf ("\n");
	// for (int i = 0; i < sensors_params[sensor_number].vertical_resolution; i++)
	// 	printf ("%lf ", sensors_params[sensor_number].vertical_correction[i]);
	// printf ("\n");

	variable_scan_update_points_with_remission_check(msg, sensors_params[sensor_number].vertical_resolution,
			&sensors_data[sensor_number].points[sensors_data[sensor_number].point_cloud_index],
			sensors_data[sensor_number].intensity[sensors_data[sensor_number].point_cloud_index],
			sensors_params[sensor_number].ray_order, sensors_params[sensor_number].vertical_correction, sensors_params[sensor_number].range_max,
			sensors_params[sensor_number].range_division_factor, msg->timestamp, use_remission);

	sensors_data[sensor_number].robot_pose[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].pose;
	sensors_data[sensor_number].robot_velocity[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].velocity;
	sensors_data[sensor_number].robot_timestamp[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].timestamp;
	sensors_data[sensor_number].robot_phi[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].phi;
	sensors_data[sensor_number].points_timestamp[sensors_data[sensor_number].point_cloud_index] = msg->timestamp;

	// if (msg_id >= 0)                // TODO ainda precisa???
	// {
	// 	//if (build_snapshot_map)
		ok_to_publish = 1;
	// 	//else

	// 	if (msg_id > 1000000)
	// 		msg_id = 0;
	// }
	// msg_id++;
	sensors_data[sensor_number].last_timestamp = msg->timestamp;   ///  ???????

	return (ok_to_publish);
}


int
mapper_stereo_velodyne_variable_scan(int sensor_number, carmen_velodyne_variable_scan_message *message)
{
	static int message_id;

	int num_points = message->number_of_shots * sensors_params[sensor_number].vertical_resolution;

	ok_to_publish = 0;
	if (!globalpos_initialized)
		return (ok_to_publish);

	if (sensors_data[sensor_number].last_timestamp == 0.0)
	{
		sensors_data[sensor_number].last_timestamp = message->timestamp;
		message_id = -2;		// antigamente eram necessarias pelo menos 2 mensagens para ter uma volta completa de velodyne

		return (ok_to_publish);
	}

	sensors_data[sensor_number].last_timestamp = sensors_data[sensor_number].current_timestamp = message->timestamp;

	build_sensor_point_cloud(&sensors_data[sensor_number].points, sensors_data[sensor_number].intensity, &sensors_data[sensor_number].point_cloud_index, num_points, NUM_VELODYNE_POINT_CLOUDS);

	carmen_velodyne_variable_scan_update_points(message, sensors_params[sensor_number].vertical_resolution,
			&sensors_data[sensor_number].points[sensors_data[sensor_number].point_cloud_index],
			sensors_data[sensor_number].intensity[sensors_data[sensor_number].point_cloud_index],
			sensors_params[sensor_number].ray_order, sensors_params[sensor_number].vertical_correction,
			sensors_params[sensor_number].range_max, message->timestamp);

	sensors_data[sensor_number].robot_pose[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].pose;
	sensors_data[sensor_number].robot_velocity[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].velocity;
	sensors_data[sensor_number].robot_timestamp[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].timestamp;
	sensors_data[sensor_number].robot_phi[sensors_data[sensor_number].point_cloud_index] = globalpos_history[last_globalpos].phi;
	sensors_data[sensor_number].points_timestamp[sensors_data[sensor_number].point_cloud_index] = message->timestamp;

	if (message_id >= 0)
	{
		ok_to_publish = 1;

		if (message_id > 1000000)
			message_id = 0;
	}
	message_id++;
	sensors_data[sensor_number].last_timestamp = message->timestamp;

	return (ok_to_publish);
}


void
mapper_merge_online_map_with_offline_map(carmen_map_set_t *map_set)
{
	for (int i = 0; i < (map_set->occupancy_map->config.x_size * map_set->occupancy_map->config.y_size); i++)
		if (map_set->occupancy_map->complete_map[i] < 0.0)
			map_set->occupancy_map->complete_map[i] = map_set->offline_map->complete_map[i];
}


void
add_virtual_laser_points(carmen_map_t *map, carmen_mapper_virtual_laser_message *virtual_laser_message)
{
	for (int i = 0; i < virtual_laser_message->num_positions; i++)
	{
		if (virtual_laser_message->colors[i] == CARMEN_PURPLE)
		{
			int x = round((virtual_laser_message->positions[i].x - map->config.x_origin) / map->config.resolution);
			int y = round((virtual_laser_message->positions[i].y - map->config.y_origin) / map->config.resolution);

			if (x >= 0 && x < map->config.x_size && y >= 0 && y < map->config.y_size)
				map->map[x][y] = 1.0;
		}
	}
//	virtual_laser_message->num_positions = 0;
}


void
draw_line(carmen_map_t *map, double x_1, double y_1, double x_2, double y_2)
{

	int x1 = round((x_1 - map->config.x_origin) / map->config.resolution);
	int y1 = round((y_1 - map->config.y_origin) / map->config.resolution);
	int x2 = round((x_2 - map->config.x_origin) / map->config.resolution);
	int y2 = round((y_2 - map->config.y_origin) / map->config.resolution);

	int w = x2 - x1;
	int h = y2 - y1;
	int dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0;

	if (w < 0)
		dx1 = -1;
	else
		if (w > 0)
			dx1 = 1;
	if (h < 0)
		dy1 = -1;
	else
		if (h > 0)
			dy1 = 1;
	if (w < 0)
		dx2 = -1;
	else
		if (w > 0)
			dx2 = 1;

	int longest = abs(w) ;
	int shortest = abs(h) ;
	if (!(longest>shortest))
	{
		longest = abs(h) ;
		shortest = abs(w) ;
		if (h < 0)
			dy2 = -1;
		else
			if (h > 0)
				dy2 = 1;
		dx2 = 0;
	}
	int numerator = longest >> 1 ;
	for (int i = 0;i <= longest;i++)
	{
		if (x1 >= 0 && x1 < map->config.x_size && y1 >= 0 && y1 < map->config.y_size)
			map->map[x1][y1] = 1.0;

		numerator += shortest ;
		if (!(numerator<longest))
		{
			numerator -= longest ;
			x1 += dx1 ;
			y1 += dy1 ;
		}
		else
		{
			x1 += dx2 ;
			y1 += dy2 ;
		}
	}
}


double
x_coord(double length2, double width2, double theta, double x)
{
	return length2 * cos(theta) - width2 *sin(theta) + x;
}


double
y_coord(double length2, double width2, double theta, double y)
{
	return length2 * sin(theta) + width2 *cos(theta) + y;
}


void
draw_rectangle(carmen_map_t *map, double x, double y, double length, double width, double theta)
{
	carmen_vector_2D_t p1, p2, p3, p4;
	double length2 = length / 2.0;
	double width2 = width / 2.0;

	p1.x = x_coord(-length2, width2, theta, x);
	p1.y = y_coord(-length2, width2, theta, y);
	p2.x = x_coord(-length2, -width2, theta, x);
	p2.y = y_coord(-length2, -width2, theta, y);
	p3.x = x_coord(length2, -width2, theta, x);
	p3.y = y_coord(length2, -width2, theta, y);
	p4.x = x_coord(length2, width2, theta, x);
	p4.y = y_coord(length2, width2, theta, y);

	draw_line(map, p1.x, p1.y, p2.x, p2.y);
	draw_line(map, p2.x, p2.y, p3.x, p3.y);
	draw_line(map, p3.x, p3.y, p4.x, p4.y);
	draw_line(map, p4.x, p4.y, p1.x, p1.y);

}


void
add_moving_objects(carmen_map_t *map, carmen_moving_objects_point_clouds_message *moving_objects_message)
{
	if (!moving_objects_message)
		return;

	for (int i = 0; i < moving_objects_message->num_point_clouds; i++)
	{
//		if (moving_objects_message->point_clouds[i].point_size)
			draw_rectangle(map,
					moving_objects_message->point_clouds[i].object_pose.x,
					moving_objects_message->point_clouds[i].object_pose.y,
					moving_objects_message->point_clouds[i].length,
					moving_objects_message->point_clouds[i].width,
					carmen_normalize_theta(moving_objects_message->point_clouds[i].orientation));
	}
}


int
carmen_parse_collision_file(double **polygon)
{
	FILE *poly;
	int n_points, h_lvl;
	char *poly_file;

	carmen_param_allow_unfound_variables(0);
	carmen_param_t param_list[] =
	{
		{ (char *) "robot", (char *) "collision_file", CARMEN_PARAM_STRING, &poly_file, 1, NULL },
	};
	carmen_param_install_params(0, NULL, param_list, sizeof(param_list) / sizeof(param_list[0]));

	poly = fopen(poly_file, "r");
	setlocale(LC_NUMERIC, "C");

	if (poly == NULL)
		printf("Can not load Col File\n");

	fscanf(poly, "%d\n", &n_points);
	fscanf(poly, "%d\n", &h_lvl);
	*polygon = (double*) malloc(n_points * 4 * sizeof(double));
	for (int i = 0; i < n_points; i++)
	{
		fscanf(poly, "%lf %lf %lf %lf\n", *polygon + (4 * i), *polygon + (4 * i + 1), *polygon + (4 * i + 2), *polygon + (4 * i + 3));
		printf("%lf %lf %lf %lf\n", (*polygon)[4 * i], (*polygon)[4 * i + 1], (*polygon)[4 * i + 2], (*polygon)[4 * i + 3]);
	}
	fclose(poly);

	return n_points;
}


int
carmen_parse_collision_semitrailer_file(double **polygon, int type)
{
	FILE *poly;
	int n_points, h_lvl;
	char *poly_file;
	char semitrailer_string[1024];
	sprintf(semitrailer_string, "semi_trailer%d_engage", type);

	carmen_param_allow_unfound_variables(0);
	carmen_param_t param_list[] =
	{
		{ semitrailer_string, (char *) "collision_file", CARMEN_PARAM_STRING, &poly_file, 1, NULL },
	};
	carmen_param_install_params(0, NULL, param_list, sizeof(param_list) / sizeof(param_list[0]));

	poly = fopen(poly_file, "r");
	setlocale(LC_NUMERIC, "C");

	if (poly == NULL)
		printf("Can not load Col Semitrailer File\n");

	fscanf(poly, "%d\n", &n_points);
	fscanf(poly, "%d\n", &h_lvl);
	*polygon = (double*) malloc(n_points * 4 * sizeof(double));
	for (int i = 0; i < n_points; i++)
	{
		fscanf(poly, "%lf %lf %lf %lf\n", *polygon + (4 * i), *polygon + (4 * i + 1), *polygon + (4 * i + 2), *polygon + (4 * i + 3));
		printf("%lf %lf %lf %lf\n", (*polygon)[4 * i], (*polygon)[4 * i + 1], (*polygon)[4 * i + 2], (*polygon)[4 * i + 3]);
	}
	fclose(poly);

	return n_points;
}


void
read_parameters_semi_trailer(carmen_semi_trailers_config_t &semi_trailer_config, int type)
{
	char semi_trailer_string[2048];

	sprintf(semi_trailer_string, "semi_trailer%d", type);

	carmen_param_t semi_trailer_param_list[] = {
		{semi_trailer_string, (char *) "d",								 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.semi_trailers[0].d),							   0, NULL},
		{semi_trailer_string, (char *) "M",								 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.semi_trailers[0].M),							   0, NULL},
		{semi_trailer_string, (char *) "width",							 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.semi_trailers[0].width),						   0, NULL},
		{semi_trailer_string, (char *) "distance_between_axle_and_front", 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.semi_trailers[0].distance_between_axle_and_front), 0, NULL},
		{semi_trailer_string, (char *) "distance_between_axle_and_back",	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.semi_trailers[0].distance_between_axle_and_back),  0, NULL},
		{semi_trailer_string, (char *) "max_beta",						 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.semi_trailers[0].max_beta),						   0, NULL},
	};
	carmen_param_install_params(0, NULL, semi_trailer_param_list, sizeof(semi_trailer_param_list)/sizeof(semi_trailer_param_list[0]));
}


void
carmen_mapper_update_cells_bellow_semitrailer(carmen_point_t pose, carmen_map_t *map, double prob, int type, double beta)
{
	static int load_polygon = 1;
	static double *collision_model_circles;
	static int col_n_points = 0;
	static carmen_semi_trailers_config_t semi_trailer_config;

	if (load_polygon)
	{
		read_parameters_semi_trailer(semi_trailer_config, type);
		col_n_points = carmen_parse_collision_semitrailer_file(&collision_model_circles, type);
		load_polygon = 0;
		printf("Col Semitrailer Loaded\n");
	}

	double collision_model_circles_center_in_world[4 * col_n_points];
	int collision_model_circles_center_in_map[4 * col_n_points];

	for (int i = 0; i < col_n_points; i++)
	{
		collision_model_circles_center_in_world[4 * i] = (collision_model_circles[4 * i] - semi_trailer_config.semi_trailers[0].d - semi_trailer_config.semi_trailers[0].M) * cos(beta) + collision_model_circles[4 * i + 1] * sin(beta);
		collision_model_circles_center_in_world[4 * i + 1] = (collision_model_circles[4 * i] - semi_trailer_config.semi_trailers[0].d - semi_trailer_config.semi_trailers[0].M) * sin(beta) - collision_model_circles[4 * i + 1] * cos(beta);
	}
	for (int i = 0; i < col_n_points; i++)
	{
		collision_model_circles_center_in_map[4 * i] = (collision_model_circles_center_in_world[4 * i] + pose.x - map->config.x_origin) / map->config.resolution;
		collision_model_circles_center_in_map[4 * i + 1] = (collision_model_circles_center_in_world[4 * i + 1] + pose.y - map->config.y_origin) / map->config.resolution;
	}
	for (int i = 0; i < col_n_points; i++)
	{
		int pix_radius = ceil(collision_model_circles[4 * i + 2] / map->config.resolution);
		for (int j = -pix_radius; j <= pix_radius; j++)
		{
			for (int k = -pix_radius; k <= pix_radius; k++)
			{
				if ((collision_model_circles_center_in_map[4 * i] + j >= 0) &&
					(collision_model_circles_center_in_map[4 * i] + j < map->config.x_size) &&
					(collision_model_circles_center_in_map[4 * i + 1] + k >= 0) &&
					(collision_model_circles_center_in_map[4 * i + 1] + k < map->config.y_size) &&
					(j * j + k * k <= pix_radius * pix_radius))
					map->complete_map[(int) (collision_model_circles_center_in_map[4 * i] + j) * map->config.y_size + (int) (collision_model_circles_center_in_map[4 * i + 1] + k)] = prob;
			}
		}
	}
}


void
carmen_mapper_update_cells_bellow_robot(carmen_point_t pose, carmen_map_t *map, double prob)
{
	static int load_polygon = 1;
	static double *collision_model_circles;
	static int col_n_points = 0;

	if (load_polygon)
	{
		col_n_points = carmen_parse_collision_file(&collision_model_circles);
		load_polygon = 0;
		printf("Col Loaded\n");
	}

	double collision_model_circles_center_in_world[4 * col_n_points];
	int collision_model_circles_center_in_map[4 * col_n_points];

	for (int i = 0; i < col_n_points; i++)
	{
		collision_model_circles_center_in_world[4 * i] = collision_model_circles[4 * i] * cos(pose.theta) + collision_model_circles[4 * i + 1] * sin(pose.theta);
		collision_model_circles_center_in_world[4 * i + 1] = collision_model_circles[4 * i] * sin(pose.theta) - collision_model_circles[4 * i + 1] * cos(pose.theta);
	}
	for (int i = 0; i < col_n_points; i++)
	{
		collision_model_circles_center_in_map[4 * i] = (collision_model_circles_center_in_world[4 * i] + pose.x - map->config.x_origin) / map->config.resolution;
		collision_model_circles_center_in_map[4 * i + 1] = (collision_model_circles_center_in_world[4 * i + 1] + pose.y - map->config.y_origin) / map->config.resolution;
	}
	for (int i = 0; i < col_n_points; i++)
	{
		int pix_radius = ceil(collision_model_circles[4 * i + 2] / map->config.resolution);
		for (int j = -pix_radius; j <= pix_radius; j++)
		{
			for (int k = -pix_radius; k <= pix_radius; k++)
			{
				if ((collision_model_circles_center_in_map[4 * i] + j >= 0) &&
					(collision_model_circles_center_in_map[4 * i] + j < map->config.x_size) &&
					(collision_model_circles_center_in_map[4 * i + 1] + k >= 0) &&
					(collision_model_circles_center_in_map[4 * i + 1] + k < map->config.y_size) &&
					(j * j + k * k <= pix_radius * pix_radius))
					map->complete_map[(int) (collision_model_circles_center_in_map[4 * i] + j) * map->config.y_size + (int) (collision_model_circles_center_in_map[4 * i + 1] + k)] = prob;
			}
		}
	}
}


void
mapper_set_robot_pose_into_the_map(carmen_map_set_t *map_set, carmen_localize_ackerman_globalpos_message *globalpos_message, int UPDATE_CELLS_BELOW_CAR)
{
	static double initial_time = 0.0;

	if (initial_time == 0.0)
		initial_time = carmen_get_time();

	if ((carmen_get_time() - initial_time) > 2.0)
		globalpos_initialized = 1;
	else
		return;

	last_globalpos = (last_globalpos + 1) % GLOBAL_POS_QUEUE_SIZE;

	globalpos_history[last_globalpos] = *globalpos_message;

	r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, globalpos_history[last_globalpos].pose.orientation);

	map_set->occupancy_map->config.x_origin = map_set->x_origin;
	map_set->occupancy_map->config.y_origin = map_set->y_origin;

	if (UPDATE_CELLS_BELOW_CAR)
	{
		carmen_mapper_update_cells_bellow_robot(globalpos_message->globalpos, map_set->occupancy_map, 0.0);
		if (globalpos_message->semi_trailer_engaged)
			carmen_mapper_update_cells_bellow_semitrailer(globalpos_message->globalpos, map_set->occupancy_map, 0.0, globalpos_message->semi_trailer_type, globalpos_message->trailer_theta[0]);
	}
}


void
mapper_update_grid_map(carmen_map_set_t *map_set, carmen_point_t xt, double *zt, sensor_parameters_t *sensor_params)
{
	carmen_update_cells_in_the_sensor_perceptual_field(map_set->occupancy_map, xt, zt, sensor_params);
}


void
mapper_save_current_map(carmen_map_set_t *map_set)
{
	if (offline_map_available && map_set->occupancy_map->complete_map != NULL && map_set->occupancy_map->config.x_origin != 0.0)
	{
		int ok = carmen_grid_mapping_save_block_map_by_origin(map_path, 'm', map_set->occupancy_map);
		if (!ok)
		{
			printf("could not write map into map_path %s\n", map_path);
			fflush(stdout);
		}

		if (use_remission)
		{
			carmen_grid_mapping_save_block_map_by_origin(map_path, 's', map_set->sum_remission_map);
			carmen_grid_mapping_save_block_map_by_origin(map_path, '2', map_set->sum_sqr_remission_map);
			carmen_grid_mapping_save_block_map_by_origin(map_path, 'c', map_set->count_remission_map);
		}

		if (create_map_sum_and_count)
		{
			carmen_grid_mapping_save_block_map_by_origin(map_path, 'u', map_set->sum_occupancy_map);
			carmen_grid_mapping_save_block_map_by_origin(map_path, 'o', map_set->count_occupancy_map);
		}
	}
}


void
mapper_periodically_save_current_map(carmen_map_set_t *map_set, double timestamp)
{
	static double last_time = timestamp;

	if ((timestamp - last_time) >= time_secs_between_map_save)
	{
		mapper_save_current_map(map_set);
		last_time = timestamp;
	}
}


carmen_map_set_t *
get_a_map_set(carmen_map_config_t map_config, bool use_merge_between_maps, bool create_map_sum_and_count, bool use_remission)
{
	carmen_map_set_t *map_set = (carmen_map_set_t *) (calloc(1, sizeof(carmen_map_set_t)));
	carmen_test_alloc(map_set);

	map_set->occupancy_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
	carmen_test_alloc(map_set->occupancy_map);
	carmen_grid_mapping_create_new_map(map_set->occupancy_map, map_config.x_size, map_config.y_size, map_config.resolution, 'm');
	map_set->new_occupancy_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
	carmen_test_alloc(map_set->new_occupancy_map);
	map_set->new_sum_occupancy_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
	carmen_test_alloc(map_set->new_sum_occupancy_map);
	map_set->new_count_occupancy_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
	carmen_test_alloc(map_set->new_count_occupancy_map);
	map_set->offline_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
	carmen_test_alloc(map_set->offline_map);
	carmen_grid_mapping_create_new_map(map_set->offline_map, map_config.x_size, map_config.y_size, map_config.resolution, 'm');

	if (use_merge_between_maps)
	{
		map_set->snapshot_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
		carmen_test_alloc(map_set->snapshot_map);
		carmen_grid_mapping_create_new_map(map_set->snapshot_map, map_config.x_size, map_config.y_size, map_config.resolution, 'm');
	}
	else
		map_set->snapshot_map = NULL;

	map_set->log_odds_snapshot_map = NULL;
	if (create_map_sum_and_count)
	{
		map_set->sum_occupancy_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
		carmen_test_alloc(map_set->sum_occupancy_map);
		map_set->count_occupancy_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
		carmen_test_alloc(map_set->count_occupancy_map);
		carmen_grid_mapping_create_new_map(map_set->sum_occupancy_map, map_config.x_size, map_config.y_size, map_config.resolution, 'u');
		carmen_grid_mapping_create_new_map(map_set->count_occupancy_map, map_config.x_size, map_config.y_size, map_config.resolution, 'o');
		//	carmen_grid_mapping_create_new_map(&variance_occupancy_map, map_config.x_size, map_config.y_size, map_config.resolution);
	}
	else
	{
		map_set->sum_occupancy_map = NULL;
		map_set->count_occupancy_map = NULL;
	}

	if (use_remission)
	{
		map_set->sum_remission_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
		carmen_test_alloc(map_set->sum_remission_map);
		map_set->sum_sqr_remission_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
		carmen_test_alloc(map_set->sum_sqr_remission_map);
		map_set->count_remission_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
		carmen_test_alloc(map_set->count_remission_map);
		carmen_grid_mapping_create_new_map(map_set->sum_remission_map, map_config.x_size, map_config.y_size, map_config.resolution, 's');
		carmen_grid_mapping_create_new_map(map_set->sum_sqr_remission_map, map_config.x_size, map_config.y_size, map_config.resolution, '2');
		carmen_grid_mapping_create_new_map(map_set->count_remission_map, map_config.x_size, map_config.y_size, map_config.resolution, 'c');

		map_set->new_sum_remission_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
		carmen_test_alloc(map_set->new_sum_remission_map);
		map_set->new_count_remission_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
		carmen_test_alloc(map_set->new_count_remission_map);
		map_set->new_sum_sqr_remission_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
		carmen_test_alloc(map_set->new_sum_sqr_remission_map);

		if (use_merge_between_maps)
		{
			map_set->sum_remission_snapshot_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
			carmen_test_alloc(map_set->sum_remission_snapshot_map);
			map_set->sum_sqr_remission_snapshot_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
			carmen_test_alloc(map_set->sum_sqr_remission_snapshot_map);
			map_set->count_remission_snapshot_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
			carmen_test_alloc(map_set->count_remission_snapshot_map);
			carmen_grid_mapping_create_new_map(map_set->sum_remission_snapshot_map, map_config.x_size, map_config.y_size, map_config.resolution, 's');
			carmen_grid_mapping_create_new_map(map_set->sum_sqr_remission_snapshot_map, map_config.x_size, map_config.y_size, map_config.resolution, '2');
			carmen_grid_mapping_create_new_map(map_set->count_remission_snapshot_map, map_config.x_size, map_config.y_size, map_config.resolution, 'c');
		}
	}
	else
	{
		map_set->sum_remission_map = NULL;
		map_set->sum_sqr_remission_map = NULL;
		map_set->count_remission_map = NULL;
		map_set->sum_remission_snapshot_map = NULL;
		map_set->sum_sqr_remission_snapshot_map = NULL;
		map_set->count_remission_snapshot_map = NULL;
	}

	map_set->moving_objects_raw_map = (carmen_map_t *) (calloc(1, sizeof(carmen_map_t)));
	carmen_test_alloc(map_set->moving_objects_raw_map);
	carmen_grid_mapping_create_new_map(map_set->moving_objects_raw_map, map_config.x_size, map_config.y_size, map_config.resolution, 'm');

	for (int i = 0; i < MAP_BUFFER_SIZE; i++)
	{
		map_set->map_buffer[i] = (carmen_map_t *) calloc(sizeof(carmen_map_t), 1);
		map_set->map_buffer[i]->complete_map = NULL;
		map_set->map_buffer[i]->map = NULL;
		map_set->map_buffer[i]->config.map_name = NULL;
	}
	map_set->map_buffer_index = 0;

	return (map_set);
}


carmen_map_set_t *
mapper_initialize(carmen_map_config_t *main_map_config, carmen_robot_ackerman_config_t main_car_config, bool use_merge_between_maps = false)
{
	car_config = main_car_config;
	map_config = *main_map_config;

	carmen_map_set_t *map_set = get_a_map_set(map_config, use_merge_between_maps, create_map_sum_and_count, use_remission);

	if (use_neural_mapper && !neural_mapper_initialized)
	{
		neural_mapper_initialized = neural_mapper_initialize(neural_mapper_max_distance_meters, neural_mapper_data_pace, map_config);
		printf("%d\n\n", neural_mapper_initialized);
	}

	globalpos_initialized = 0; // Only considered initialized when first message is received

	globalpos_history = (carmen_localize_ackerman_globalpos_message *) calloc(GLOBAL_POS_QUEUE_SIZE, sizeof(carmen_localize_ackerman_globalpos_message));
	carmen_test_alloc(globalpos_history);

	memset(&last_rddf_annotation_message, 0, sizeof(last_rddf_annotation_message));

	memset(&virtual_laser_message, 0, sizeof(carmen_mapper_virtual_laser_message));
//	virtual_laser_message.positions = (carmen_position_t *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(carmen_position_t));
//	carmen_test_alloc(virtual_laser_message.positions);
//	virtual_laser_message.colors = (char *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(char));
//	carmen_test_alloc(virtual_laser_message.colors);
//	virtual_laser_message.host = carmen_get_host();

	memset(&virtual_scan_message, 0, sizeof(carmen_mapper_virtual_scan_message));
	virtual_scan_message.host = carmen_get_host();

	last_globalpos = 0;

	return (map_set);
}


void
carmen_mapper_initialize_transforms()
{
	tf::Transform board_to_gps_pose;
	tf::Transform board_to_camera_pose;
	tf::Transform car_to_board_pose;
	tf::Transform world_to_car_pose;
	tf::Transform ultrasonic_sensor_r1_to_car_pose;
	tf::Transform ultrasonic_sensor_r2_to_car_pose;
	tf::Transform ultrasonic_sensor_l1_to_car_pose;
	tf::Transform ultrasonic_sensor_l2_to_car_pose;

	tf::Time::init();

	// initial car pose with respect to the world
	world_to_car_pose.setOrigin(tf::Vector3(car_pose_g.position.x, car_pose_g.position.y, car_pose_g.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(car_pose_g.orientation.yaw, car_pose_g.orientation.pitch, car_pose_g.orientation.roll));
	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	tf_transformer.setTransform(world_to_car_transform, "world_to_car_transform");

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(sensor_board_1_pose.position.x, sensor_board_1_pose.position.y, sensor_board_1_pose.position.z));
	car_to_board_pose.setRotation(tf::Quaternion(sensor_board_1_pose.orientation.yaw, sensor_board_1_pose.orientation.pitch, sensor_board_1_pose.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	tf_transformer.setTransform(car_to_board_transform, "car_to_board_transform");

	// gps pose with respect to the board
	board_to_gps_pose.setOrigin(tf::Vector3(gps_pose_in_the_car.position.x, gps_pose_in_the_car.position.y, gps_pose_in_the_car.position.z));
	board_to_gps_pose.setRotation(tf::Quaternion(gps_pose_in_the_car.orientation.yaw, gps_pose_in_the_car.orientation.pitch, gps_pose_in_the_car.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform board_to_gps_transform(board_to_gps_pose, tf::Time(0), "/board", "/gps");
	tf_transformer.setTransform(board_to_gps_transform, "board_to_gps_transform");

	// camera pose with respect to the board
	board_to_camera_pose.setOrigin(tf::Vector3(camera_pose_g.position.x, camera_pose_g.position.y, camera_pose_g.position.z));
	board_to_camera_pose.setRotation(tf::Quaternion(camera_pose_g.orientation.yaw, camera_pose_g.orientation.pitch, camera_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform board_to_camera_transform(board_to_camera_pose, tf::Time(0), "/board", "/camera");
	tf_transformer.setTransform(board_to_camera_transform, "board_to_camera_transform");

	// initial ultrasonic sensor r1 pose with respect to the car
	ultrasonic_sensor_r1_to_car_pose.setOrigin(tf::Vector3(ultrasonic_sensor_r1_g.position.x, ultrasonic_sensor_r1_g.position.y, ultrasonic_sensor_r1_g.position.z));
	ultrasonic_sensor_r1_to_car_pose.setRotation(tf::Quaternion(ultrasonic_sensor_r1_g.orientation.yaw, ultrasonic_sensor_r1_g.orientation.pitch, ultrasonic_sensor_r1_g.orientation.roll));
	tf::StampedTransform ultrasonic_sensor_r1_to_car_transform(ultrasonic_sensor_r1_to_car_pose, tf::Time(0), "/car", "/ultrasonic_sensor_r1");
	tf_transformer.setTransform(ultrasonic_sensor_r1_to_car_transform, "ultrasonic_sensor_r1_to_car_transform");

	// initial ultrasonic sensor r2 pose with respect to the car
	ultrasonic_sensor_r2_to_car_pose.setOrigin(tf::Vector3(ultrasonic_sensor_r2_g.position.x, ultrasonic_sensor_r2_g.position.y, ultrasonic_sensor_r2_g.position.z));
	ultrasonic_sensor_r2_to_car_pose.setRotation(tf::Quaternion(ultrasonic_sensor_r2_g.orientation.yaw, ultrasonic_sensor_r2_g.orientation.pitch, ultrasonic_sensor_r2_g.orientation.roll));
	tf::StampedTransform ultrasonic_sensor_r2_to_car_transform(ultrasonic_sensor_r2_to_car_pose, tf::Time(0), "/car", "/ultrasonic_sensor_r2");
	tf_transformer.setTransform(ultrasonic_sensor_r2_to_car_transform, "ultrasonic_sensor_r2_to_car_transform");

	// initial ultrasonic sensor l2 pose with respect to the car
	ultrasonic_sensor_l2_to_car_pose.setOrigin(tf::Vector3(ultrasonic_sensor_l2_g.position.x, ultrasonic_sensor_l2_g.position.y, ultrasonic_sensor_l2_g.position.z));
	ultrasonic_sensor_l2_to_car_pose.setRotation(tf::Quaternion(ultrasonic_sensor_l2_g.orientation.yaw, ultrasonic_sensor_l2_g.orientation.pitch, ultrasonic_sensor_l2_g.orientation.roll));
	tf::StampedTransform ultrasonic_sensor_l2_to_car_transform(ultrasonic_sensor_l2_to_car_pose, tf::Time(0), "/car", "/ultrasonic_sensor_l2");
	tf_transformer.setTransform(ultrasonic_sensor_l2_to_car_transform, "ultrasonic_sensor_l2_to_car_transform");

	// initial ultrasonic sensor l1 pose with respect to the car
	ultrasonic_sensor_l1_to_car_pose.setOrigin(tf::Vector3(ultrasonic_sensor_l1_g.position.x, ultrasonic_sensor_l1_g.position.y, ultrasonic_sensor_l1_g.position.z));
	ultrasonic_sensor_l1_to_car_pose.setRotation(tf::Quaternion(ultrasonic_sensor_l1_g.orientation.yaw, ultrasonic_sensor_l1_g.orientation.pitch, ultrasonic_sensor_l1_g.orientation.roll));
	tf::StampedTransform ultrasonic_sensor_l1_to_car_transform(ultrasonic_sensor_l1_to_car_pose, tf::Time(0), "/car", "/ultrasonic_sensor_l1");
	tf_transformer.setTransform(ultrasonic_sensor_l1_to_car_transform, "ultrasonic_sensor_l1_to_car_transform");
}


void
carmen_mapper_get_highest_sensor()
{
	highest_sensor = 0.0;

	for (int i = 0; i < number_of_sensors; i++)
		if (sensors_params[i].alive && sensors_params[i].height > highest_sensor)
			highest_sensor = sensors_params[i].height;
}


void
carmen_mapper_override_mapping_mode_params(int argc, char **argv)
{
	if (mapping_mode == 0)
	{
		carmen_param_t param_list[] =
		{
			{(char *) "mapper",  (char *) "mapping_mode_off_update_and_merge_with_snapshot_map", CARMEN_PARAM_ONOFF, &update_and_merge_with_snapshot_map, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_off_merge_with_offline_map", CARMEN_PARAM_ONOFF, &merge_with_offline_map, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_off_decay_to_offline_map", CARMEN_PARAM_ONOFF, &decay_to_offline_map, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_off_update_and_merge_with_mapper_saved_maps", CARMEN_PARAM_ONOFF, &update_and_merge_with_mapper_saved_maps, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_off_build_snapshot_map", CARMEN_PARAM_ONOFF, &build_snapshot_map, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_off_velodyne_range_max", CARMEN_PARAM_DOUBLE, &mapper_velodyne_range_max, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_off_velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &mapper_range_max_factor, 1, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_off_create_map_sum_and_count", CARMEN_PARAM_ONOFF, &create_map_sum_and_count, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_off_use_remission", CARMEN_PARAM_ONOFF, &use_remission, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_off_laser_ldmrs", CARMEN_PARAM_ONOFF, &sensors_params[1].alive, 1, carmen_mapper_sensors_params_handler},
			{(char *) "mapper",  (char *) "mapping_mode_off_use_merge_between_maps", CARMEN_PARAM_ONOFF, &use_merge_between_maps, 0, NULL},
		};
		carmen_param_allow_unfound_variables(0);
		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

		carmen_param_t optional_param_list[] =
		{
			{(char *) "mapper",  (char *) "mapping_mode_off_min_force_obstacle_height", CARMEN_PARAM_DOUBLE, &min_force_obstacle_height, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_off_max_force_obstacle_height", CARMEN_PARAM_DOUBLE, &max_force_obstacle_height, 0, NULL},
		};

		carmen_param_allow_unfound_variables(1);
		carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));
	}
	else if (mapping_mode == 1)
	{
		carmen_param_t param_list[] =
		{
			{(char *) "mapper",  (char *) "mapping_mode_on_update_and_merge_with_snapshot_map", CARMEN_PARAM_ONOFF, &update_and_merge_with_snapshot_map, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_on_merge_with_offline_map", CARMEN_PARAM_ONOFF, &merge_with_offline_map, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_on_decay_to_offline_map", CARMEN_PARAM_ONOFF, &decay_to_offline_map, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_on_update_and_merge_with_mapper_saved_maps", CARMEN_PARAM_ONOFF, &update_and_merge_with_mapper_saved_maps, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_on_build_snapshot_map", CARMEN_PARAM_ONOFF, &build_snapshot_map, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_on_velodyne_range_max", CARMEN_PARAM_DOUBLE, &mapper_velodyne_range_max, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_on_velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &mapper_range_max_factor, 1, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_on_create_map_sum_and_count", CARMEN_PARAM_ONOFF, &create_map_sum_and_count, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_on_use_remission", CARMEN_PARAM_ONOFF, &use_remission, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_on_laser_ldmrs", CARMEN_PARAM_ONOFF, &sensors_params[1].alive, 1, carmen_mapper_sensors_params_handler},
			{(char *) "mapper",  (char *) "mapping_mode_on_use_merge_between_maps", CARMEN_PARAM_ONOFF, &use_merge_between_maps, 0, NULL},

		};
		carmen_param_allow_unfound_variables(0);
		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

		carmen_param_t optional_param_list[] =
		{
			{(char *) "mapper",  (char *) "mapping_mode_on_min_force_obstacle_height", CARMEN_PARAM_DOUBLE, &min_force_obstacle_height, 0, NULL},
			{(char *) "mapper",  (char *) "mapping_mode_on_max_force_obstacle_height", CARMEN_PARAM_DOUBLE, &max_force_obstacle_height, 0, NULL},
		};

		carmen_param_allow_unfound_variables(1);
		carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));
	}

	if (level_msg == 1)
	{
		carmen_param_t param_list[] =
		{
			{(char *) "mapper",  (char *) "level1_update_and_merge_with_snapshot_map", CARMEN_PARAM_ONOFF, &update_and_merge_with_snapshot_map, 0, NULL},
			{(char *) "mapper",  (char *) "level1_merge_with_offline_map", CARMEN_PARAM_ONOFF, &merge_with_offline_map, 0, NULL},
			{(char *) "mapper",  (char *) "level1_decay_to_offline_map", CARMEN_PARAM_ONOFF, &decay_to_offline_map, 0, NULL},
			{(char *) "mapper",  (char *) "level1_update_and_merge_with_mapper_saved_maps", CARMEN_PARAM_ONOFF, &update_and_merge_with_mapper_saved_maps, 0, NULL},
			{(char *) "mapper",  (char *) "level1_build_snapshot_map", CARMEN_PARAM_ONOFF, &build_snapshot_map, 0, NULL},
			{(char *) "mapper",  (char *) "level1_velodyne_range_max", CARMEN_PARAM_DOUBLE, &mapper_velodyne_range_max, 0, NULL},
			{(char *) "mapper",  (char *) "level1_velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &mapper_range_max_factor, 1, NULL},
			{(char *) "mapper",  (char *) "level1_create_map_sum_and_count", CARMEN_PARAM_ONOFF, &create_map_sum_and_count, 0, NULL},
			{(char *) "mapper",  (char *) "level1_use_remission", CARMEN_PARAM_ONOFF, &use_remission, 0, NULL},
			{(char *) "mapper",  (char *) "level1_laser_ldmrs", CARMEN_PARAM_ONOFF, &sensors_params[1].alive, 1, carmen_mapper_sensors_params_handler},
			{(char *) "mapper",  (char *) "level1_use_merge_between_maps", CARMEN_PARAM_ONOFF, &use_merge_between_maps, 0, NULL},

		};
		carmen_param_allow_unfound_variables(1);
		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		carmen_param_allow_unfound_variables(0);
	}
}


void
carmen_mapper_sensors_params_handler(char *module, char *variable, __attribute__((unused)) char *value)
{
	if (strcmp(module, "mapper") == 0)
	{
		//char lidar_string_id[32];

		int i = number_of_sensors;

		// if (strcmp(variable, "unsafe_height_above_ground") == 0)
		// {
		// 	for (i = 1; i < number_of_sensors; i++)
		// 	{
		// 		sensors_params[i].unsafe_height_above_ground = sensors_params[0].unsafe_height_above_ground;
		// 	}
		// 	return;
		// }

		if (strcmp(variable, "velodyne") == 0)
			i = 0;
		else if (strcmp(variable, "laser_ldmrs") == 0)
			i = 1;
		else if (strncmp(variable, "stereo_velodyne", 15) == 0 && strlen(variable) == 16)
			i = variable[15] - '0';

		// else if (strncmp(variable, "lidar", 5) == 0 && (strlen(variable) == 6 || strlen(variable) == 7))
		// {
		// 	if(strlen(variable) == 6)
		// 		i = variable[5] - '0';
		// 	if(strlen(variable) == 7)
		// 	{
		// 		sprintf(lidar_string_id, "%d%d", variable[5], variable[6]);
		// 		i = atoi(lidar_string_id);
		// 	}
		// }

		if (i < number_of_sensors && sensors_params[i].alive && sensors_params[i].name == NULL)
		{
			sensors_params[i].name = (char *) calloc(strlen(variable) + 1, sizeof(char));
			strcpy(sensors_params[i].name, variable);
			return;
		}
	}
}


void
carmen_mapper_read_alive_lidars_configs(int argc, char **argv)
{
	char locc_string[64], lfree_string[64], l0_string[64], unexpeted_delta_range_sigma_string[64];
	carmen_lidar_config *p;

	for (int i = 0; i < MAX_NUMBER_OF_LIDARS; i++)
	{
		if (sensors_params[i + 10].alive)  // Lidars start from 10 in the sensors_params vector
		{
			p = &lidar_config[i];
			load_lidar_config(argc, argv, i, &p);

			sprintf(locc_string, "lidar%d_locc", i);
			sprintf(lfree_string, "lidar%d_lfree", i);
			sprintf(l0_string, "lidar%d_l0", i);
			sprintf(unexpeted_delta_range_sigma_string, "lidar%d_unexpeted_delta_range_sigma", i);

			carmen_param_t param_list[] = {
				{(char *) "mapper", (char *) locc_string, CARMEN_PARAM_DOUBLE, &sensors_params[i + 10].log_odds.log_odds_occ, 1, NULL},
				{(char *) "mapper", (char *) lfree_string, CARMEN_PARAM_DOUBLE, &sensors_params[i + 10].log_odds.log_odds_free, 1, NULL},
				{(char *) "mapper", (char *) l0_string, CARMEN_PARAM_DOUBLE, &sensors_params[i + 10].log_odds.log_odds_l0, 1, NULL},
				{(char *) "mapper", (char *) unexpeted_delta_range_sigma_string, CARMEN_PARAM_DOUBLE, &sensors_params[i + 10].unexpeted_delta_range_sigma, 1, NULL},
			};
			carmen_param_install_params(argc, argv, param_list, (sizeof(param_list) / sizeof(param_list[0])));
		}
	}
}


void
carmen_mapper_get_alive_sensors(int argc, char **argv)
{
	int i;

	sensors_params = (sensor_parameters_t *) calloc(number_of_sensors, sizeof(sensor_parameters_t));
	carmen_test_alloc(sensors_params);

	sensors_data = (sensor_data_t *) calloc(number_of_sensors, sizeof(sensor_data_t));
	carmen_test_alloc(sensors_data);

	carmen_param_t param_list[] =
	{
		{(char *) "mapper", (char *) "velodyne", CARMEN_PARAM_ONOFF, &sensors_params[0].alive, 1, carmen_mapper_sensors_params_handler},
		{(char *) "mapper", (char *) "laser_ldmrs", CARMEN_PARAM_ONOFF, &sensors_params[1].alive, 1, carmen_mapper_sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne2", CARMEN_PARAM_ONOFF, &sensors_params[2].alive, 1, carmen_mapper_sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne3", CARMEN_PARAM_ONOFF, &sensors_params[3].alive, 1, carmen_mapper_sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne4", CARMEN_PARAM_ONOFF, &sensors_params[4].alive, 1, carmen_mapper_sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne5", CARMEN_PARAM_ONOFF, &sensors_params[5].alive, 1, carmen_mapper_sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne6", CARMEN_PARAM_ONOFF, &sensors_params[6].alive, 1, carmen_mapper_sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne7", CARMEN_PARAM_ONOFF, &sensors_params[7].alive, 1, carmen_mapper_sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne8", CARMEN_PARAM_ONOFF, &sensors_params[8].alive, 1, carmen_mapper_sensors_params_handler},
		{(char *) "mapper", (char *) "stereo_velodyne9", CARMEN_PARAM_ONOFF, &sensors_params[9].alive, 1, carmen_mapper_sensors_params_handler},
		//{(char *) "mapper", (char *) "stereo_mapping", CARMEN_PARAM_ONOFF, &sensors_params[STEREO_MAPPING_SENSOR_INDEX].alive, 1, carmen_mapper_sensors_params_handler},

		{(char *) "mapper", (char *) "lidar0", CARMEN_PARAM_ONOFF, &sensors_params[10].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar1", CARMEN_PARAM_ONOFF, &sensors_params[11].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar2", CARMEN_PARAM_ONOFF, &sensors_params[12].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar3", CARMEN_PARAM_ONOFF, &sensors_params[13].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar4", CARMEN_PARAM_ONOFF, &sensors_params[14].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar5", CARMEN_PARAM_ONOFF, &sensors_params[15].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar6", CARMEN_PARAM_ONOFF, &sensors_params[16].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar7", CARMEN_PARAM_ONOFF, &sensors_params[17].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar8", CARMEN_PARAM_ONOFF, &sensors_params[18].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar9", CARMEN_PARAM_ONOFF, &sensors_params[19].alive, 1, NULL},
        {(char *) "mapper", (char *) "lidar10", CARMEN_PARAM_ONOFF, &sensors_params[20].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar11", CARMEN_PARAM_ONOFF, &sensors_params[21].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar12", CARMEN_PARAM_ONOFF, &sensors_params[22].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar13", CARMEN_PARAM_ONOFF, &sensors_params[23].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar14", CARMEN_PARAM_ONOFF, &sensors_params[24].alive, 1, NULL},
		{(char *) "mapper", (char *) "lidar15", CARMEN_PARAM_ONOFF, &sensors_params[25].alive, 1, NULL},

		{(char *) "mapper", (char *) "lidar8_locc", CARMEN_PARAM_ONOFF, &sensors_params[18].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "velodyne_locc", CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "laser_ldmrs_locc", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne2_locc", CARMEN_PARAM_DOUBLE, &sensors_params[2].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne3_locc", CARMEN_PARAM_DOUBLE, &sensors_params[3].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne4_locc", CARMEN_PARAM_DOUBLE, &sensors_params[4].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne5_locc", CARMEN_PARAM_DOUBLE, &sensors_params[5].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne6_locc", CARMEN_PARAM_DOUBLE, &sensors_params[6].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne7_locc", CARMEN_PARAM_DOUBLE, &sensors_params[7].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne8_locc", CARMEN_PARAM_DOUBLE, &sensors_params[8].log_odds.log_odds_occ, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne9_locc", CARMEN_PARAM_DOUBLE, &sensors_params[9].log_odds.log_odds_occ, 1, NULL},
//			{(char *) "mapper", (char *) "stereo_mapping_locc", CARMEN_PARAM_DOUBLE, &sensors_params[STEREO_MAPPING_SENSOR_INDEX].log_odds.log_odds_occ, 1, NULL},

		{(char *) "mapper", (char *) "lidar8_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[18].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "velodyne_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "laser_ldmrs_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_free, 1, NULL},
//			{(char *) "mapper", (char *) "stereo_velodyne1_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne2_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[2].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne3_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[3].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne4_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[4].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne5_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[5].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne6_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[6].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne7_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[7].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne8_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[8].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne9_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[9].log_odds.log_odds_free, 1, NULL},
//			{(char *) "mapper", (char *) "stereo_mapping_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[STEREO_MAPPING_SENSOR_INDEX].log_odds.log_odds_free, 1, NULL},
		{(char *) "mapper", (char *) "lidar8_l0", CARMEN_PARAM_DOUBLE, &sensors_params[18].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "velodyne_l0", CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "laser_ldmrs_l0", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_l0, 1, NULL},
//			{(char *) "mapper", (char *) "stereo_velodyne1_l0", CARMEN_PARAM_DOUBLE, &sensors_params[1].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne2_l0", CARMEN_PARAM_DOUBLE, &sensors_params[2].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne3_l0", CARMEN_PARAM_DOUBLE, &sensors_params[3].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne4_l0", CARMEN_PARAM_DOUBLE, &sensors_params[4].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne5_l0", CARMEN_PARAM_DOUBLE, &sensors_params[5].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne6_l0", CARMEN_PARAM_DOUBLE, &sensors_params[6].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne7_l0", CARMEN_PARAM_DOUBLE, &sensors_params[7].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne8_l0", CARMEN_PARAM_DOUBLE, &sensors_params[8].log_odds.log_odds_l0, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne9_l0", CARMEN_PARAM_DOUBLE, &sensors_params[9].log_odds.log_odds_l0, 1, NULL},

		{(char *) "mapper", (char *) "lidar8_ray_index_difference", CARMEN_PARAM_DOUBLE, &sensors_params[18].ray_index_difference, 1, NULL},
		{(char *) "mapper", (char *) "lidar8_use_index_difference", CARMEN_PARAM_DOUBLE, &sensors_params[18].use_index_difference, 1, NULL},
		{(char *) "mapper", (char *) "lidar8_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[18].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "velodyne_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[0].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "laser_ldmrs_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[1].unexpeted_delta_range_sigma, 1, NULL},
//			{(char *) "mapper", (char *) "stereo_velodyne1_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[1].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne2_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[2].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne3_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[3].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne4_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[4].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne5_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[5].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne6_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[6].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne7_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[7].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne8_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[8].unexpeted_delta_range_sigma, 1, NULL},
		{(char *) "mapper", (char *) "stereo_velodyne9_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[9].unexpeted_delta_range_sigma, 1, NULL},

		// {(char *) "mapper", (char *) "unsafe_height_above_ground", CARMEN_PARAM_DOUBLE, &mapper_unsafe_height_above_ground, 1, carmen_mapper_sensors_params_handler},
		// {(char *) "mapper",  (char *) "velodyne_range_max_factor", CARMEN_PARAM_DOUBLE, &sensors_params[0].range_max_factor, 1, NULL}
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	for (i = 0; i < number_of_sensors; i++)
	{
		sensors_params[i].unsafe_height_above_ground = mapper_unsafe_height_above_ground;

		sensors_data[i].ray_position_in_the_floor = (carmen_vector_2D_t **)  calloc(number_of_threads, sizeof(carmen_vector_2D_t*));
		sensors_data[i].maxed = (int **) calloc(number_of_threads, sizeof(int*));
		sensors_data[i].obstacle_height = (double **) calloc(number_of_threads, sizeof(double*));
		sensors_data[i].occupancy_log_odds_of_each_ray_target = (double **) calloc(number_of_threads, sizeof(double*));
		sensors_data[i].point_cloud_index = -1;
		sensors_data[i].points = NULL;
		sensors_data[i].ray_origin_in_the_floor = (carmen_vector_2D_t **) calloc(number_of_threads, sizeof(carmen_vector_2D_t*));
		sensors_data[i].ray_size_in_the_floor = (double **) calloc(number_of_threads, sizeof(double*));
		sensors_data[i].processed_intensity = (double **) calloc(number_of_threads, sizeof(double*));
		sensors_data[i].ray_hit_the_robot = (int **) calloc(number_of_threads, sizeof(int*));
		sensors_data[i].ray_that_hit_the_nearest_target = (int *) calloc(number_of_threads, sizeof(int));

		sensors_params[i].name = NULL;
		sensors_params[i].ray_order = NULL;
		sensors_params[i].sensor_to_support_matrix = NULL;
		sensors_params[i].vertical_correction = NULL;
		sensors_params[i].vertical_resolution = 0;

		for (int j = 0; j < number_of_threads; j++)
		{
			sensors_data[i].ray_position_in_the_floor[j] = NULL;
			sensors_data[i].maxed[j] = NULL;
			sensors_data[i].obstacle_height[j] = NULL;
			sensors_data[i].occupancy_log_odds_of_each_ray_target[j] = NULL;
			sensors_data[i].ray_origin_in_the_floor[j] = NULL;
			sensors_data[i].ray_size_in_the_floor[j] = NULL;
			sensors_data[i].processed_intensity[j] = NULL;
			sensors_data[i].ray_hit_the_robot[j] = NULL;
		}

		if (sensors_params[i].alive)
		{
			sensors_params[i].name = (char *) calloc(strlen(param_list[i].variable) + 1, sizeof(char));
			strcpy(sensors_params[i].name, param_list[i].variable);
		}
	}
}


void
read_parameters_semi_trailer(int semi_trailer_type)
{
	semi_trailer_config.num_semi_trailers = semi_trailer_type;
	if (semi_trailer_type == 0)
		return;

	for (int semi_trailer_id=1; semi_trailer_id <= semi_trailer_config.num_semi_trailers; semi_trailer_id++)
	{
		char semi_trailer_string[2048];

		sprintf(semi_trailer_string, "%s%d", "semi_trailer", semi_trailer_id);

		char *semi_trailer_poly_file;

		carmen_param_t semi_trailer_param_list[] = {
			{semi_trailer_string, (char *) "d",								 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.semi_trailers[semi_trailer_id-1].d),							   	  0, NULL},
			{semi_trailer_string, (char *) "M",								 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.semi_trailers[semi_trailer_id-1].M),							   	  0, NULL},
			{semi_trailer_string, (char *) "width",							 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.semi_trailers[semi_trailer_id-1].width),							  0, NULL},
			{semi_trailer_string, (char *) "distance_between_axle_and_front", 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_front), 0, NULL},
			{semi_trailer_string, (char *) "distance_between_axle_and_back",	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.semi_trailers[semi_trailer_id-1].distance_between_axle_and_back),  0, NULL},
			{semi_trailer_string, (char *) "max_beta",						 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.semi_trailers[semi_trailer_id-1].max_beta),						  0, NULL},
			{semi_trailer_string, (char *) "polygon_file",					 	CARMEN_PARAM_STRING, &(semi_trailer_poly_file), 							  	0, NULL}
		};
		carmen_param_install_params(g_argc, g_argv, semi_trailer_param_list, sizeof(semi_trailer_param_list)/sizeof(semi_trailer_param_list[0]));

		semi_trailer_config.semi_trailers[semi_trailer_id-1].max_beta = carmen_degrees_to_radians(semi_trailer_config.semi_trailers[semi_trailer_id-1].max_beta);
	}
}


void
carmen_mapper_init_velodyne_points(spherical_point_cloud **velodyne_points_out, unsigned char ***intencity, carmen_pose_3D_t **robot_pose_out,
		carmen_vector_3D_t **robot_velocity_out, double **robot_timestamp_out, double **robot_phi_out, double **points_timestamp_out)
{
	int i;

	carmen_pose_3D_t *robot_pose = (carmen_pose_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_pose_3D_t));
	carmen_vector_3D_t *robot_velocity = (carmen_vector_3D_t *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(carmen_vector_3D_t));
	spherical_point_cloud *velodyne_points = (spherical_point_cloud *)malloc(NUM_VELODYNE_POINT_CLOUDS * sizeof(spherical_point_cloud));
	double *robot_timestamp = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));
	*intencity = (unsigned char **)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(unsigned char *));
	*robot_phi_out = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));
	*points_timestamp_out = (double *)calloc(NUM_VELODYNE_POINT_CLOUDS, sizeof(double));


	carmen_test_alloc(velodyne_points);

	for (i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
	{
		velodyne_points[i].num_points = 0;
		velodyne_points[i].sphere_points = NULL;
	}

	*velodyne_points_out = velodyne_points;
	*robot_pose_out = robot_pose;
	*robot_velocity_out = robot_velocity;
	*robot_timestamp_out = robot_timestamp;
}


int *
carmen_mapper_generates_ray_order(int size)
{
	int *ray_order = (int *) malloc(size * sizeof(int));
	carmen_test_alloc(ray_order);

	for (int i = 0; i < size; i++)
		ray_order[i] = i;

	return (ray_order);
}


void
carmen_mapper_sort_ray_order_by_vertical_correction_angles(sensor_parameters_t params)
{
	for (int i = params.vertical_resolution - 1; i > 0; i--)
	{
		for (int j = 0; j < i; j++)
		{
			if (params.vertical_correction[params.ray_order[j]] > params.vertical_correction[params.ray_order[j + 1]])
			{
				int aux = params.ray_order[j];
				params.ray_order[j] = params.ray_order[j + 1];
				params.ray_order[j + 1] = aux;
			}
		}
	}
}


void
carmen_mapper_sort_vertical_correction_angles(sensor_parameters_t params)
{
	double aux[params.vertical_resolution];

	memcpy(aux, params.vertical_correction, params.vertical_resolution * sizeof(double));

	for (int i = 0; i < params.vertical_resolution; i++)
		params.vertical_correction[i] = aux[params.ray_order[i]];
}


void
carmen_mapper_get_lidars_sensor_params()
{
#ifdef USE_REAR_BULLBAR
	//0 a 2, 0 é a sensor_board, 1 é a front_bullbar, 2 é a rear_bullbar
	carmen_pose_3D_t choosed_sensor_referenced[] = {sensor_board_1_pose, front_bullbar_pose, rear_bullbar_pose};
#endif

	for (int i = 0; i < MAX_NUMBER_OF_LIDARS; i++)
	{
		if (!sensors_params[i + 10].alive)
			continue;
		
		if (calibration_file)
			sensors_params[i + 10].calibration_table = load_calibration_table(calibration_file);
		else
			sensors_params[i + 10].calibration_table = load_calibration_table((char *) "calibration_table.txt");

		if (save_calibration_file)
			sensors_params[i + 10].save_calibration_file = fopen(save_calibration_file, "w"); // The file is closed in the shutdown_module handler
		else
			sensors_params[i + 10].save_calibration_file = NULL;

		sensors_params[i + 10].name = lidar_config[i].model;
		sensors_params[i + 10].pose = lidar_config[i].pose;

#ifdef USE_REAR_BULLBAR
		sensors_params[i + 10].sensor_reference = lidar_config[i].sensor_reference;
		sensors_params[i + 10].sensor_support_pose = choosed_sensor_referenced[sensors_params[i + 10].sensor_reference];
#else
		sensors_params[i + 10].sensor_support_pose = sensor_board_1_pose;
#endif

		sensors_params[i + 10].support_to_car_matrix = create_rotation_matrix(sensors_params[i + 10].sensor_support_pose.orientation);
		
		sensors_params[i + 10].sensor_to_support_matrix = create_rotation_matrix(sensors_params[i + 10].pose.orientation);

		sensors_params[i + 10].sensor_robot_reference = carmen_change_sensor_reference(sensors_params[i + 10].sensor_support_pose.position, 
		                                                    sensors_params[i + 10].pose.position, sensors_params[i + 10].support_to_car_matrix);

		sensors_params[i + 10].height = sensors_params[i + 10].sensor_robot_reference.z + robot_wheel_radius;

		sensors_params[i + 10].sensor_type = VELODYNE;
		sensors_params[i + 10].vertical_resolution = lidar_config[i].shot_size;
		sensors_params[i + 10].ray_order = lidar_config[i].ray_order;
		sensors_params[i + 10].vertical_correction = lidar_config[i].vertical_angles;
		sensors_params[i + 10].range_max = lidar_config[i].max_range;
		sensors_params[i + 10].range_max_factor = mapper_range_max_factor;
		sensors_params[i + 10].range_division_factor = (double) lidar_config[i].range_division_factor;
		sensors_params[i + 10].time_spent_by_each_scan = lidar_config[i].time_between_shots;
		sensors_params[i + 10].unsafe_height_above_ground = mapper_unsafe_height_above_ground;

		carmen_mapper_init_velodyne_points(&sensors_data[i + 10].points, &sensors_data[i + 10].intensity, &sensors_data[i + 10].robot_pose, &sensors_data[i + 10].robot_velocity,
		        &sensors_data[i + 10].robot_timestamp, &sensors_data[i + 10].robot_phi, &sensors_data[i + 10].points_timestamp);
		
		sensors_params[i + 10].current_range_max = sensors_params[i + 10].range_max;
		
		sensors_data[i + 10].point_cloud_index = -1;

		carmen_prob_models_alloc_sensor_data(&sensors_data[i + 10], sensors_params[i + 10].vertical_resolution, number_of_threads);

		sensors_params[i + 10].remission_calibration = NULL; //(double *) calloc(256 * sensors_params[0].vertical_resolution, sizeof(double));

		carmen_mapper_sort_ray_order_by_vertical_correction_angles(sensors_params[i + 10]);
		carmen_mapper_sort_vertical_correction_angles(sensors_params[i + 10]);
#ifdef USE_REAR_BULLBAR
		sensors_params[i + 10].sensor_reference = lidar_config[i].sensor_reference;
#endif

	}
}


void
carmen_mapper_get_sensors_param(int argc, char **argv)
{
	//-----------------------------------------------------------------------------------------//
	// The Velodyne HDL32E is hard coded as sensor 0 when using velodyne_partial_scan_message
	if (calibration_file)
		sensors_params[0].calibration_table = load_calibration_table(calibration_file);
	else
		sensors_params[0].calibration_table = load_calibration_table((char *) "calibration_table.txt");

	if (save_calibration_file)
		sensors_params[0].save_calibration_file = fopen(save_calibration_file, "w"); // Eh fechado em shutdown_module()
	else
		sensors_params[0].save_calibration_file = NULL;

	sensors_params[0].pose = velodyne_pose;
	sensors_params[0].sensor_support_pose = sensor_board_1_pose;
	sensors_params[0].support_to_car_matrix = create_rotation_matrix(sensors_params[0].sensor_support_pose.orientation);
	sensors_params[0].sensor_robot_reference = carmen_change_sensor_reference(
			sensors_params[0].sensor_support_pose.position, sensors_params[0].pose.position, sensors_params[0].support_to_car_matrix);
	sensors_params[0].height = sensors_params[0].sensor_robot_reference.z + robot_wheel_radius;

	if (sensors_params[0].height > highest_sensor)
		highest_sensor = sensors_params[0].height;

	if (sensors_params[0].alive && !strcmp(sensors_params[0].name, "velodyne"))
	{
		sensors_params[0].sensor_type = VELODYNE;
		sensors_params[0].ray_order = carmen_velodyne_get_ray_order();
		sensors_params[0].range_max_factor = mapper_range_max_factor;
		sensors_params[0].vertical_correction = carmen_velodyne_get_vertical_correction();
		// sensors_params[0].delta_difference_mean = carmen_velodyne_get_delta_difference_mean();       // Ninguem sabe para que servem estes valores e aparentemente nao sao usados para nada
		// sensors_params[0].delta_difference_stddev = carmen_velodyne_get_delta_difference_stddev();

		carmen_param_t param_list[] =
		{
			{sensors_params[0].name, (char *) "vertical_resolution", CARMEN_PARAM_INT, &sensors_params[0].vertical_resolution, 0, NULL},
			{(char *) "mapper", (char *) "velodyne_range_max", CARMEN_PARAM_DOUBLE, &sensors_params[0].range_max, 0, NULL},
			{sensors_params[0].name, (char *) "time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &sensors_params[0].time_spent_by_each_scan, 0, NULL},
		};

		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		carmen_mapper_init_velodyne_points(&sensors_data[0].points, &sensors_data[0].intensity, &sensors_data[0].robot_pose, &sensors_data[0].robot_velocity, &sensors_data[0].robot_timestamp, &sensors_data[0].robot_phi, &sensors_data[0].points_timestamp);
		sensors_params[0].sensor_to_support_matrix = create_rotation_matrix(sensors_params[0].pose.orientation);
		sensors_params[0].current_range_max = sensors_params[0].range_max;
		sensors_data[0].point_cloud_index = -1;
		carmen_prob_models_alloc_sensor_data(&sensors_data[0], sensors_params[0].vertical_resolution, number_of_threads);

		sensors_params[0].unsafe_height_above_ground = mapper_unsafe_height_above_ground;

		sensors_params[0].remission_calibration = NULL;//(double *) calloc(256 * sensors_params[0].vertical_resolution, sizeof(double));
//		FILE *f = fopen("../data/remission_calibration.txt", "r");
//		for (i = 0; i < 256 * sensors_params[0].vertical_resolution; i++)
//		{
//			fscanf(f, "%lf", &sensors_params[0].remission_calibration[i]);
//		}
//		fclose(f);
	}

	//-------------------------------------------------------------------------------//
	// The SICK LDMRS is hard coded as sensor 1
	sensors_params[1].calibration_table = NULL;
	sensors_params[1].save_calibration_file = NULL;

	if (sensors_params[1].alive && !strcmp(sensors_params[1].name, "laser_ldmrs"))
	{
		sensors_params[1].sensor_type = LASER_LDMRS;
		sensors_params[1].pose = laser_ldmrs_pose;
		sensors_params[1].sensor_support_pose = front_bullbar_pose;
		sensors_params[1].support_to_car_matrix = create_rotation_matrix(sensors_params[1].sensor_support_pose.orientation);
		sensors_params[1].sensor_robot_reference = carmen_change_sensor_reference(
				sensors_params[1].sensor_support_pose.position, sensors_params[1].pose.position, sensors_params[1].support_to_car_matrix);
		sensors_params[1].height = sensors_params[1].sensor_robot_reference.z + robot_wheel_radius;

		if (sensors_params[1].height > highest_sensor)
			highest_sensor = sensors_params[1].height;

		static int ray_order[4] = {0, 1, 2, 3};
		sensors_params[1].ray_order = ray_order;

		static double vertical_correction[4] = {-1.2, -0.4, 0.4, 1.2};
		sensors_params[1].vertical_correction = vertical_correction;

		// static double delta_difference_mean[4] = {0, 0, 0, 0};       // Ninguem sabe para que servem estes valores e aparentemente nao sao usados para nada
		// sensors_params[1].delta_difference_mean = delta_difference_mean;

		// static double delta_difference_stddev[4] = {0, 0, 0, 0};
		// sensors_params[1].delta_difference_stddev = delta_difference_stddev;

		sensors_params[1].range_max_factor = 1.0;

		carmen_param_t param_list[] =
		{
			{(char *) "laser_ldmrs", (char *) "vertical_resolution", CARMEN_PARAM_INT, &sensors_params[1].vertical_resolution, 0, NULL},
			{(char *) "laser_ldmrs", (char *) "range_max", CARMEN_PARAM_DOUBLE, &sensors_params[1].range_max, 0, NULL},
			{(char *) "laser_ldmrs", (char *) "time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &sensors_params[1].time_spent_by_each_scan, 0, NULL},
			{(char *) "laser_ldmrs", (char *) "cutoff_negative_acceleration", CARMEN_PARAM_DOUBLE, &sensors_params[1].cutoff_negative_acceleration, 0, NULL},
		};

		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		carmen_mapper_init_velodyne_points(&sensors_data[1].points, &sensors_data[1].intensity, &sensors_data[1].robot_pose, &sensors_data[1].robot_velocity, &sensors_data[1].robot_timestamp, &sensors_data[1].robot_phi, &sensors_data[1].points_timestamp);
		sensors_params[1].sensor_to_support_matrix = create_rotation_matrix(sensors_params[1].pose.orientation);
		sensors_params[1].current_range_max = sensors_params[1].range_max;
		sensors_data[1].point_cloud_index = -1;
		carmen_prob_models_alloc_sensor_data(&sensors_data[1], sensors_params[1].vertical_resolution, number_of_threads);

		sensors_params[1].unsafe_height_above_ground = mapper_unsafe_height_above_ground;

		sensors_params[1].remission_calibration = NULL;//(double *) calloc(256 * sensors_params[0].vertical_resolution, sizeof(double));
	}

	//-------------------------------------------------------------------------------//
	// STEREO CAMERAS
	for (int i = 2; i < 10; i++)
	{
		sensors_params[i].calibration_table = NULL;
		sensors_params[i].save_calibration_file = NULL;

		if (sensors_params[i].alive)
		{
			sensors_params[i].sensor_type = CAMERA;
			sensors_params[i].pose = get_stereo_velodyne_pose_3D(argc, argv, i);
			sensors_params[i].sensor_support_pose = sensor_board_1_pose;
			sensors_params[i].support_to_car_matrix = create_rotation_matrix(sensors_params[i].sensor_support_pose.orientation);
			sensors_params[i].sensor_robot_reference = carmen_change_sensor_reference(
					sensors_params[i].sensor_support_pose.position, sensors_params[i].pose.position, sensors_params[i].support_to_car_matrix);
			sensors_params[i].height = sensors_params[i].sensor_robot_reference.z + robot_wheel_radius;

			if (sensors_params[i].height > highest_sensor)
				highest_sensor = sensors_params[i].height;

			sensors_params[i].time_spent_by_each_scan = 0.0;

			char stereo_velodyne_string[256];
			sprintf(stereo_velodyne_string, "%s%d", "stereo", i);

			int horizontal_resolution;
			int flipped;
			int stereo_velodyne_vertical_roi_ini;
			int stereo_velodyne_vertical_roi_end;
			int stereo_velodyne_horizontal_roi_ini;
			int stereo_velodyne_horizontal_roi_end;
			carmen_param_t param_list[] =
			{
				{sensors_params[i].name, (char *) "vertical_resolution", CARMEN_PARAM_INT, &sensors_params[i].vertical_resolution, 0, NULL},
				{sensors_params[i].name, (char *) "horizontal_resolution", CARMEN_PARAM_INT, &horizontal_resolution, 0, NULL},
				{sensors_params[i].name, (char *) "flipped", CARMEN_PARAM_ONOFF, &flipped, 0, NULL},
				{sensors_params[i].name, (char *) "range_max", CARMEN_PARAM_DOUBLE, &sensors_params[i].range_max, 0, NULL},
				{sensors_params[i].name, (char *) "vertical_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_ini, 0, NULL },
				{sensors_params[i].name, (char *) "vertical_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_vertical_roi_end, 0, NULL },
				{sensors_params[i].name, (char *) "horizontal_roi_ini", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_ini, 0, NULL },
				{sensors_params[i].name, (char *) "horizontal_roi_end", CARMEN_PARAM_INT, &stereo_velodyne_horizontal_roi_end, 0, NULL }
			};

			carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

			int roi_ini, roi_end;
			if (flipped)
			{
				sensors_params[i].vertical_resolution = horizontal_resolution;
				roi_ini = stereo_velodyne_horizontal_roi_ini;
				roi_end = stereo_velodyne_horizontal_roi_end;
			}
			else
			{
				roi_ini = stereo_velodyne_vertical_roi_ini;
				roi_end = stereo_velodyne_vertical_roi_end;
			}

			if (sensors_params[i].vertical_resolution > (roi_end - roi_ini))
			{
				carmen_die("The stereo_velodyne_vertical_resolution is bigger than stereo point cloud height");
			}
			sensors_params[i].range_max_factor = mapper_range_max_factor;
			sensors_params[i].ray_order = carmen_mapper_generates_ray_order(sensors_params[i].vertical_resolution);
			sensors_params[i].vertical_correction = get_stereo_velodyne_correction(flipped, i, sensors_params[i].vertical_resolution, roi_ini, roi_end, 0, 0);
			carmen_mapper_init_velodyne_points(&sensors_data[i].points, &sensors_data[i].intensity, &sensors_data[i].robot_pose, &sensors_data[i].robot_velocity,  &sensors_data[i].robot_timestamp, &sensors_data[i].robot_phi, &sensors_data[i].points_timestamp);
			sensors_params[i].sensor_to_support_matrix = create_rotation_matrix(sensors_params[i].pose.orientation);
			sensors_params[i].current_range_max = sensors_params[i].range_max;
			sensors_data[i].point_cloud_index = -1;
			carmen_prob_models_alloc_sensor_data(&sensors_data[i], sensors_params[i].vertical_resolution, number_of_threads);

			sensors_params[i].unsafe_height_above_ground = mapper_unsafe_height_above_ground;

			// sensors_params[i].delta_difference_mean = (double *)calloc(50, sizeof(double));
			// sensors_params[i].delta_difference_stddev = (double *)calloc(50, sizeof(double));
			// for (j = 0; j < 50; j++)
			// 	sensors_params[i].delta_difference_stddev[j] = 1.0;
		}
	}
}


void
get_sensors_param_pose_handler(__attribute__((unused)) char *module, __attribute__((unused)) char *variable, __attribute__((unused)) char *value)
{
	if (sensors_params[0].alive && strcmp(sensors_params[0].name, "velodyne") == 0)
	{
		sensors_params[0].pose = velodyne_pose;
		sensors_params[0].sensor_support_pose = sensor_board_1_pose;
		sensors_params[0].support_to_car_matrix = create_rotation_matrix(sensors_params[0].sensor_support_pose.orientation);
		sensors_params[0].sensor_robot_reference = carmen_change_sensor_reference(
				sensors_params[0].sensor_support_pose.position, sensors_params[0].pose.position, sensors_params[0].support_to_car_matrix);
		sensors_params[0].height = sensors_params[0].sensor_robot_reference.z + robot_wheel_radius;
	}
	else if (sensors_params[1].alive && strcmp(sensors_params[1].name, "laser_ldmrs") != 0)
	{
		sensors_params[1].sensor_support_pose = sensor_board_1_pose;
		free(sensors_params[1].support_to_car_matrix);
		sensors_params[1].support_to_car_matrix = create_rotation_matrix(sensors_params[1].sensor_support_pose.orientation);
		sensors_params[1].sensor_robot_reference = carmen_change_sensor_reference(
				sensors_params[1].sensor_support_pose.position, sensors_params[1].pose.position, sensors_params[1].support_to_car_matrix);
		sensors_params[1].height = sensors_params[1].sensor_robot_reference.z + robot_wheel_radius;
	} 
}


// void
// get_occupancy_log_odds_of_each_ray_target(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int scan_index)
// {
// 	int thread_id = omp_get_thread_num();
// 	int point_cloud_index = sensor_data->point_cloud_index;
// 	spherical_point_cloud v_zt = sensor_data->points[point_cloud_index];
// 	int N = v_zt.num_points / sensor_params->vertical_resolution;
// 	double v = sensor_data->robot_velocity[point_cloud_index].x;
// 	double phi = sensor_data->robot_phi[point_cloud_index];
// 	double dt = sensor_params->time_spent_by_each_scan;
// 	double dt1 = sensor_data->points_timestamp[point_cloud_index] - sensor_data->robot_timestamp[point_cloud_index] - (double) N * dt;
// 	carmen_pose_3D_t robot_interpolated_position = sensor_data->robot_pose[point_cloud_index];
// 	carmen_pose_3D_t robot_pose = sensor_data->robot_pose[point_cloud_index];
// 	double dt2 = dt * scan_index / sensor_params->vertical_resolution;
// 	robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(robot_pose,
// 			dt1 + dt2, v, phi, car_config.distance_between_front_and_rear_axles);

// 	r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

// 	change_sensor_rear_range_max(sensor_params, v_zt.sphere_points[scan_index].horizontal_angle);

// 	carmen_prob_models_compute_relevant_map_coordinates_with_remission_check(sensor_data, sensor_params, scan_index, robot_interpolated_position.position,
// 			sensor_params->sensor_support_pose, r_matrix_car_to_global, sensor_params->support_to_car_matrix,
// 			robot_wheel_radius, offline_map.config.x_origin, offline_map.config.y_origin, &car_config, robot_near_strong_slow_down_annotation, thread_id, use_remission);

// 	carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(sensor_data, sensor_params, scan_index, highest_sensor, safe_range_above_sensors,
// 			robot_near_strong_slow_down_annotation, thread_id);
// 			// Updates: sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][ray_index] : 0 <= ray_index < 32
// }


// void
// carmen_mapper_fill_lidar_point_cloud_each_ray_hit_obstacle_probabily_message(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int camera_index, int image_index)
// {
// 	int cloud_index = sensor_data->point_cloud_index;
// 	int number_of_laser_shots = sensor_data->points[cloud_index].num_points / sensor_params->vertical_resolution;
// 	int thread_id = omp_get_thread_num();

// 	for (int j = 0; j < number_of_laser_shots; j++)
// 	{
// 		int scan_index = j * sensor_params->vertical_resolution;
// 		double horizontal_angle = - sensor_data->points[cloud_index].sphere_points[scan_index].horizontal_angle;

// 		get_occupancy_log_odds_of_each_ray_target(sensor_params, sensor_data, scan_index);

// 		for (int i = 1; i < sensor_params->vertical_resolution; i++)
// 		{
// 			double vertical_angle = sensor_data->points[cloud_index].sphere_points[scan_index + i].vertical_angle;
// 			double range = sensor_data->points[cloud_index].sphere_points[scan_index + i].length;

// 			double log_odds = sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i];
// 			double prob = carmen_prob_models_log_odds_to_probabilistic(log_odds);
// 			//prob contem a probabilidade de um raio ter atingido um obstáculo ou não, no mapper tudo com prob>0.5 atingiu um obstáculo
			
// 		}
// 	}
// }


// read all parameters from .ini file and command line
void
carmen_mapper_read_parameters(int argc, char **argv, carmen_map_config_t *map_config, carmen_robot_ackerman_config_t *p_car_config)
{
	double robot_vertical_displacement_from_center;

	double map_resolution, map_width, map_height;
	carmen_param_t param_list[] =
	{
		{(char *) "robot",  (char *) "distance_between_front_car_and_front_wheels", 	CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_front_car_and_front_wheels), 1, NULL},
		{(char *) "robot",  (char *) "distance_between_front_and_rear_axles",		CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_front_and_rear_axles), 1, NULL},
		{(char *) "robot",  (char *) "distance_between_rear_car_and_rear_wheels",		CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_rear_car_and_rear_wheels), 1, NULL},
		{(char *) "robot",  (char *) "distance_between_rear_wheels",			CARMEN_PARAM_DOUBLE, &(p_car_config->distance_between_rear_wheels), 1, NULL},
		{(char *) "robot",  (char *) "length", CARMEN_PARAM_DOUBLE, &p_car_config->length, 0, NULL},
		{(char *) "robot",  (char *) "width", CARMEN_PARAM_DOUBLE, &p_car_config->width, 0, NULL},
		{(char *) "robot",  (char *) "vertical_displacement_from_center", CARMEN_PARAM_DOUBLE, &robot_vertical_displacement_from_center, 0, NULL},
		{(char *) "robot",  (char *) "wheel_radius", CARMEN_PARAM_DOUBLE, &(robot_wheel_radius), 0, NULL},

		{(char *) "model", 		(char *) "predictive_planner_obstacles_safe_distance", 	CARMEN_PARAM_DOUBLE, &(p_car_config->model_predictive_planner_obstacles_safe_distance), 1, NULL},
		{(char *) "obstacle", 	(char *) "avoider_obstacles_safe_distance", 			CARMEN_PARAM_DOUBLE, &(p_car_config->obstacle_avoider_obstacles_safe_distance), 1, NULL},

		{(char *) "semi_trailer", (char *) "initial_type", CARMEN_PARAM_INT, &(semi_trailer_config.num_semi_trailers), 0, NULL},

		{(char *) "sensor",  (char *) "board_1_x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x),	1, get_sensors_param_pose_handler},
		{(char *) "sensor",  (char *) "board_1_y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y),	1, get_sensors_param_pose_handler},
		{(char *) "sensor",  (char *) "board_1_z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z),	1, get_sensors_param_pose_handler},
		{(char *) "sensor",  (char *) "board_1_roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll),   1, get_sensors_param_pose_handler},
		{(char *) "sensor",  (char *) "board_1_pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch), 1, get_sensors_param_pose_handler},
		{(char *) "sensor",  (char *) "board_1_yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw),	  1, get_sensors_param_pose_handler},

		{(char *) "front_bullbar",  (char *) "x", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.position.x),	0, NULL},
		{(char *) "front_bullbar",  (char *) "y", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.position.y),	0, NULL},
		{(char *) "front_bullbar",  (char *) "z", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.position.z),	0, NULL},
		{(char *) "front_bullbar",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.orientation.roll),0, NULL},
		{(char *) "front_bullbar",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.orientation.pitch),0, NULL},
		{(char *) "front_bullbar",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(front_bullbar_pose.orientation.yaw),	0, NULL},

#ifdef USE_REAR_BULLBAR
		{(char *) "rear_bullbar",  (char *) "x", CARMEN_PARAM_DOUBLE, &(rear_bullbar_pose.position.x),	0, NULL},
		{(char *) "rear_bullbar",  (char *) "y", CARMEN_PARAM_DOUBLE, &(rear_bullbar_pose.position.y),	0, NULL},
		{(char *) "rear_bullbar",  (char *) "z", CARMEN_PARAM_DOUBLE, &(rear_bullbar_pose.position.z),	0, NULL},
		{(char *) "rear_bullbar",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(rear_bullbar_pose.orientation.roll),0, NULL},
		{(char *) "rear_bullbar",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(rear_bullbar_pose.orientation.pitch),0, NULL},
		{(char *) "rear_bullbar",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(rear_bullbar_pose.orientation.yaw),	0, NULL},
#endif
		{(char *) "velodyne",  (char *) "x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 1, get_sensors_param_pose_handler},
		{(char *) "velodyne",  (char *) "y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 1, get_sensors_param_pose_handler},
		{(char *) "velodyne",  (char *) "z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 1, get_sensors_param_pose_handler},
		{(char *) "velodyne",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll),   1, get_sensors_param_pose_handler},
		{(char *) "velodyne",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 1, get_sensors_param_pose_handler},
		{(char *) "velodyne",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw),     1, get_sensors_param_pose_handler},

		{(char *) "laser_ldmrs",  (char *) "x", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.position.x), 0, NULL},
		{(char *) "laser_ldmrs",  (char *) "y", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.position.y), 0, NULL},
		{(char *) "laser_ldmrs",  (char *) "z", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.position.z), 0, NULL},
		{(char *) "laser_ldmrs",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.orientation.roll), 0, NULL},
		{(char *) "laser_ldmrs",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.orientation.pitch), 0, NULL},
		{(char *) "laser_ldmrs",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(laser_ldmrs_pose.orientation.yaw), 0, NULL},

		{(char *) "gps", (char *) "nmea_1_x",		CARMEN_PARAM_DOUBLE, &gps_pose_in_the_car.position.x,		1, NULL},
		{(char *) "gps", (char *) "nmea_1_y",		CARMEN_PARAM_DOUBLE, &gps_pose_in_the_car.position.y,		1, NULL},
		{(char *) "gps", (char *) "nmea_1_z",		CARMEN_PARAM_DOUBLE, &gps_pose_in_the_car.position.z,		1, NULL},
		{(char *) "gps", (char *) "nmea_1_roll",	CARMEN_PARAM_DOUBLE, &gps_pose_in_the_car.orientation.roll,		1, NULL},
		{(char *) "gps", (char *) "nmea_1_pitch",	CARMEN_PARAM_DOUBLE, &gps_pose_in_the_car.orientation.pitch,	1, NULL},
		{(char *) "gps", (char *) "nmea_1_yaw",		CARMEN_PARAM_DOUBLE, &gps_pose_in_the_car.orientation.yaw,		1, NULL},

		{(char *) "mapper",  (char *) "number_of_sensors", CARMEN_PARAM_INT, &number_of_sensors, 0, NULL}, // The number_of_sensors must be the maximun number of sensors: 25
		{(char *) "mapper",  (char *) "safe_range_above_sensors", CARMEN_PARAM_DOUBLE, &safe_range_above_sensors, 0, NULL},
		{(char *) "mapper",  (char *) "safe_height_from_ground", CARMEN_PARAM_DOUBLE, &safe_height_from_ground, 0, NULL},

		{(char *) "mapper",  (char *) "map_grid_res", CARMEN_PARAM_DOUBLE, &map_resolution, 0, NULL},
		{(char *) "mapper",  (char *) "map_width", CARMEN_PARAM_DOUBLE, &map_width, 0, NULL},
		{(char *) "mapper",  (char *) "map_height", CARMEN_PARAM_DOUBLE, &map_height, 0, NULL},
		{(char *) "mapper",  (char *) "unsafe_height_above_ground", CARMEN_PARAM_DOUBLE, &mapper_unsafe_height_above_ground, 0, NULL},
		{(char *) "mapper",  (char *) "moving_objects_raw_map", CARMEN_PARAM_ONOFF, &publish_moving_objects_raw_map, 0, NULL},
		{(char *) "mapper",  (char *) "build_snapshot_map", CARMEN_PARAM_ONOFF, &build_snapshot_map, 0, NULL},
		{(char *) "mapper",  (char *) "merge_with_offline_map", CARMEN_PARAM_ONOFF, &merge_with_offline_map, 0, NULL},
		{(char *) "mapper",  (char *) "update_and_merge_with_mapper_saved_maps", CARMEN_PARAM_ONOFF, &update_and_merge_with_mapper_saved_maps, 0, NULL},
		{(char *) "mapper",  (char *) "update_cells_below_car", CARMEN_PARAM_ONOFF, &update_cells_below_car, 0, NULL},
		{(char *) "mapper",  (char *) "decay_to_offline_map", CARMEN_PARAM_ONOFF, &decay_to_offline_map, 0, NULL},
		{(char *) "mapper",  (char *) "create_map_sum_and_count", CARMEN_PARAM_ONOFF, &create_map_sum_and_count, 0, NULL},
		{(char *) "mapper",  (char *) "save_map", CARMEN_PARAM_ONOFF, &mapper_save_map, 0, NULL},
		{(char *) "mapper",  (char *) "rays_threshold_to_merge_between_maps", CARMEN_PARAM_DOUBLE, &rays_threshold_to_merge_between_maps, 0, NULL},
		{(char *) "mapper",  (char *) "update_and_merge_with_snapshot_map", CARMEN_PARAM_ONOFF, &update_and_merge_with_snapshot_map, 0, NULL},
		{(char *) "mapper",  (char *) "number_of_threads", CARMEN_PARAM_INT, &number_of_threads, 0, NULL},

		{(char *) "commandline",  (char *) "map_path", CARMEN_PARAM_STRING, &map_path, 0, NULL},

		{(char *) "visual_odometry", (char *) "is_global_pos", CARMEN_PARAM_ONOFF, &visual_odometry_is_global_pos, 0, NULL},

		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_sampling_step", CARMEN_PARAM_INT, &ultrasonic_sensor_params.sampling_step, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_num_beams", CARMEN_PARAM_INT, &ultrasonic_sensor_params.laser_beams, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_fov_range", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.fov_range, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_max_range", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.range_max, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_lambda_short", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.lambda_short, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_sigma_zhit", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.sigma_zhit, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_zhit", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.zhit, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_zmax", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.zmax, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_zrand", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.zrand, 0, NULL},
		{(char *) "grid_mapping", (char *) "ultrasonic_sensor_zshort", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.zshort, 0, NULL},

		{(char *) "grid_mapping", (char *) "map_locc", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.log_odds.log_odds_occ, 0, NULL},
		{(char *) "grid_mapping", (char *) "map_lfree", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.log_odds.log_odds_free, 0, NULL},
		{(char *) "grid_mapping", (char *) "map_l0", CARMEN_PARAM_DOUBLE, &ultrasonic_sensor_params.log_odds.log_odds_l0, 0, NULL},

		{(char *) "ultrasonic_sensor_r1", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.position.x), 0, NULL},
		{(char *) "ultrasonic_sensor_r1", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.position.y), 0, NULL},
		{(char *) "ultrasonic_sensor_r1", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.position.z), 0, NULL},
		{(char *) "ultrasonic_sensor_r1", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.orientation.yaw), 0, NULL},
		{(char *) "ultrasonic_sensor_r1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.orientation.pitch), 0, NULL},
		{(char *) "ultrasonic_sensor_r1", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r1_g.orientation.roll), 0, NULL},

		{(char *) "ultrasonic_sensor_r2", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.position.x), 0, NULL},
		{(char *) "ultrasonic_sensor_r2", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.position.y), 0, NULL},
		{(char *) "ultrasonic_sensor_r2", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.position.z), 0, NULL},
		{(char *) "ultrasonic_sensor_r2", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.orientation.yaw), 0, NULL},
		{(char *) "ultrasonic_sensor_r2", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.orientation.pitch), 0, NULL},
		{(char *) "ultrasonic_sensor_r2", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_r2_g.orientation.roll), 0, NULL},

		{(char *) "ultrasonic_sensor_l2", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.position.x), 0, NULL},
		{(char *) "ultrasonic_sensor_l2", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.position.y), 0, NULL},
		{(char *) "ultrasonic_sensor_l2", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.position.z), 0, NULL},
		{(char *) "ultrasonic_sensor_l2", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.orientation.yaw), 0, NULL},
		{(char *) "ultrasonic_sensor_l2", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.orientation.pitch), 0, NULL},
		{(char *) "ultrasonic_sensor_l2", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l2_g.orientation.roll), 0, NULL},

		{(char *) "ultrasonic_sensor_l1", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.position.x), 0, NULL},
		{(char *) "ultrasonic_sensor_l1", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.position.y), 0, NULL},
		{(char *) "ultrasonic_sensor_l1", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.position.z), 0, NULL},
		{(char *) "ultrasonic_sensor_l1", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.orientation.yaw), 0, NULL},
		{(char *) "ultrasonic_sensor_l1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.orientation.pitch), 0, NULL},
		{(char *) "ultrasonic_sensor_l1", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(ultrasonic_sensor_l1_g.orientation.roll), 0, NULL},

		{(char *) "behavior_selector", 	  (char *) "use_truepos", CARMEN_PARAM_ONOFF, &use_truepos, 0, NULL},

		{(char *) "frenet_path_planner",  (char *) "use_unity_simulator", CARMEN_PARAM_ONOFF, &use_unity_simulator, 0, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	int mkdir_status = mkdir(map_path, 0775);
	if (mkdir_status != 0)
	{
		if (errno != EEXIST)
			carmen_die("ERROR: mapper could not create new directory '%s'\n", map_path);

		struct stat map_path_stat;
		int map_path_status = stat(map_path, &map_path_stat);
		if (map_path_status != 0 || !S_ISDIR(map_path_stat.st_mode))
			carmen_die("ERROR: mapper could not create new directory '%s'\n", map_path);
	}

	ultrasonic_sensor_params.current_range_max = ultrasonic_sensor_params.range_max;

	if (semi_trailer_config.num_semi_trailers > 0)
		read_parameters_semi_trailer(semi_trailer_config.num_semi_trailers);

	if (map_width != map_height)
		carmen_die("Wrong map size: width (%f) must be equal to height (%f).", map_width, map_height);

	if ((((int) map_width) % 3) != 0)
		carmen_die("Wrong map size: width (%f) and height (%f) must be multiple of 3.", map_width, map_height);

	map_config->x_size = round(map_width / map_resolution);
	map_config->y_size = round(map_height / map_resolution);
	map_config->resolution = map_resolution;

	carmen_grid_mapping_init_parameters(map_resolution, map_width);

	// GET THE ALIVE SENSORS FROM IN THE .INI FILE
	carmen_mapper_get_alive_sensors(argc, argv);

	// READ CONFIGS OF LIDARS THAT ARE ALIVE (ON IN THE .ini FILE)
	carmen_mapper_read_alive_lidars_configs(argc, argv);

	carmen_param_allow_unfound_variables(1);

	carmen_param_t param_optional_list[] =
	{
		{(char *) "commandline", (char *) "calibration_file", CARMEN_PARAM_STRING, &calibration_file, 0, NULL},
		{(char *) "commandline", (char *) "save_calibration_file", CARMEN_PARAM_STRING, &save_calibration_file, 0, NULL},
		{(char *) "commandline", (char *) "use_neural_mapper", CARMEN_PARAM_ONOFF, &use_neural_mapper, 0, NULL},
		{(char *) "commandline", (char *) "generate_neural_mapper_dataset", CARMEN_PARAM_ONOFF, &generate_neural_mapper_dataset, 0, NULL},
		{(char *) "commandline", (char *) "neural_mapper_max_distance_meters", CARMEN_PARAM_INT, &neural_mapper_max_distance_meters, 0, NULL},
		{(char *) "commandline", (char *) "neural_mapper_data_pace", CARMEN_PARAM_INT, &neural_mapper_data_pace, 0, NULL},
		{(char *) "commandline", (char *) "num_clouds", CARMEN_PARAM_INT, &neural_map_num_clouds, 0, NULL},
		{(char *) "commandline", (char *) "time_secs_between_map_save", CARMEN_PARAM_DOUBLE, &time_secs_between_map_save, 0, NULL},
		{(char *) "commandline", (char *) "mapping_mode", CARMEN_PARAM_ONOFF, &mapping_mode, 0, NULL},
		{(char *) "commandline", (char *) "level_msg", CARMEN_PARAM_INT, &level_msg, 0, NULL},
		{(char *) "commandline", (char *) "clean_bellow_car", CARMEN_PARAM_ONOFF, &clean_map_bellow_car, 0, NULL},
		{(char *) "mapper", 	 (char *) "publish_diff_map",		   CARMEN_PARAM_ONOFF,  &publish_diff_map, 		    1, NULL},
		{(char *) "mapper", 	 (char *) "publish_diff_map_interval", CARMEN_PARAM_DOUBLE, &publish_diff_map_interval, 1, NULL},
		{(char *) "mapper", 	 (char *) "safe_height_from_ground_level1", CARMEN_PARAM_DOUBLE, &safe_height_from_ground_level, 0, NULL},
	};
	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));

	if (mapping_mode == 0 || mapping_mode == 1)
		carmen_mapper_override_mapping_mode_params(argc, argv);
	
	if (level_msg == 1)
		safe_height_from_ground = safe_height_from_ground_level;

	if (clean_map_bellow_car)
		update_and_merge_with_mapper_saved_maps = 1;

	// FILL THE sensor_params STRUCTURE VECTOR
	carmen_mapper_get_sensors_param(argc, argv);

	// FILL THE sensor_params STRUCTURE VECTOR from 10 to 25 with alive lidars data
	carmen_mapper_get_lidars_sensor_params();
}

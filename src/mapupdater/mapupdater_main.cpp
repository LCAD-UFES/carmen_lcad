#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/base_ackerman_messages.h>
#include <carmen/base_ackerman_interface.h>
#include <prob_measurement_model.h>
#include <prob_motion_model.h>
#include <prob_monte_carlo.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/rotation_geometry.h>
#include <carmen/velodyne_interface.h>
#include <carmen/mapper_interface.h>
#include <carmen/localize_ackerman_core.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_velodyne.h>
#include <carmen/xsens_interface.h>
#include <carmen/mapper.h>

#include "fastslam.h"

carmen_map_config_t map_config;

carmen_base_ackerman_odometry_message base_ackerman_odometry_vector[BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE];
carmen_fused_odometry_message fused_odometry_vector[FUSED_ODOMETRY_VECTOR_SIZE];
int g_fused_odometry_index = -1;

carmen_robot_ackerman_config_t 	car_config;
carmen_semi_trailers_config_t 	semi_trailer_config;

carmen_map_t local_map;
carmen_map_t local_sum_remission_map;
carmen_map_t local_mean_remission_map;
carmen_map_t local_variance_remission_map;
carmen_map_t local_sum_sqr_remission_map;
carmen_map_t local_count_remission_map;

carmen_compact_map_t local_compacted_map;
carmen_compact_map_t local_compacted_mean_remission_map;
carmen_compact_map_t local_compacted_variance_remission_map;
carmen_localize_ackerman_binary_map_t binary_map;

carmen_behavior_selector_path_goals_and_annotations_message *behavior_selector_path_goals_and_annotations_message = NULL;

carmen_localize_ackerman_globalpos_message globalpos = {};

carmen_localize_ackerman_param_t localize_param;

int update_and_merge_with_mapper_saved_maps;
int update_and_merge_with_snapshot_map;
int decay_to_offline_map;
int create_map_sum_and_count;
int use_remission;
int build_snapshot_map;

carmen_rddf_annotation_message last_rddf_annotation_message;
int robot_near_strong_slow_down_annotation = 0;
rotation_matrix *r_matrix_car_to_global = NULL;

int use_neural_mapper = 0;
int generate_neural_mapper_dataset = 0;
int neural_mapper_max_distance_meters = 0;
int neural_mapper_data_pace = 0;
int use_unity_simulator = 0;

int ok_to_publish = 0;
bool offline_map_available = true;
carmen_localize_ackerman_globalpos_message *globalpos_history;
int last_globalpos = 0;
double time_secs_between_map_save = 0.0;

char *map_path;
char *map_path_swap;

double robot_wheel_radius;
carmen_pose_3D_t velodyne_pose;
int number_of_sensors;
double safe_range_above_sensors;
double safe_height_from_ground;
int level_msg;
double safe_height_from_ground_level;

double highest_sensor = 0.0;

int number_of_threads = 1;

sensor_parameters_t *sensors_params;
sensor_data_t *sensors_data;

int mapping_mode = 1;

extern int globalpos_initialized;

extern bool use_merge_between_maps;
extern int mapper_save_map;

int best_particle_index = 0;

//extern tf::Transformer tf_transformer;

double initial_angle = 1000.0;

bool request_to_final_save = false;

double log_playback_speed = 1.0;
double log_time_between_likelihood_maps_update = 1.0;

//extern carmen_velodyne_partial_scan_message *last_velodyne_message;

carmen_localize_ackerman_particle_filter_p filter;	// So para guardar paramentros e tambem passa-los para bibliotecas que assumem este global.

// Map server
static double initial_waiting_time = 3.0;

static int block_map = 1;
static double map_grid_res;
static double map_width;
static double map_height;
static int initial_map_x;
static int initial_map_y;

static char *map_file_name = NULL;

double lane_width = 0.0;

static int publish_grid_mapping_map_at_startup = 0;
static int publish_google_map = 0;
static int lanemap_incoming_message_type = 0;
static int goal_source_path_planner = 0;
static int rddf_source_tracker = 0;

static double time_interval_for_map_change = 0.3;

static carmen_localize_ackerman_map_t localize_map;

int offline_map_published = 0;

int is_first_rddf_message = 1;
carmen_rddf_road_profile_message *rddf_message = NULL;

static carmen_point_t pose_g;

static double distance_to_update_lane_map = 1.0;

double mapupdater_percentage_change_for_update;
double mapupdater_max_log_odds;
double mapupdater_min_log_odds;
double mapupdater_max_count;
double mapupdater_strenght_dacay;

carmen_map_set_t *current_map_set;
carmen_map_set_t *updated_offline_map_set;


///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_map(double timestamp)
{
	// Publica o mapa compactado apenas com as celulas com probabilidade igual ou maior que 0.5
//	carmen_compact_map_t cmap;
//	carmen_prob_models_create_compact_map_with_cells_larger_than_value(&cmap, &occupancy_map, 0.5);
//	carmen_mapper_publish_compact_map_message(&cmap, timestamp);
//	carmen_prob_models_free_compact_map(&cmap);

	// Publica o mapa nao compactado
//	carmen_mapper_publish_map_message(&occupancy_map, timestamp);

	// A publicacao apenas para visualizacao
	carmen_moving_objects_map_message moving_objects_map_message;
	moving_objects_map_message.complete_map = current_map_set->occupancy_map->complete_map;
	moving_objects_map_message.size = current_map_set->occupancy_map->config.x_size * current_map_set->occupancy_map->config.y_size;
	moving_objects_map_message.config = current_map_set->occupancy_map->config;
	moving_objects_map_message.timestamp = timestamp;
	moving_objects_map_message.host = carmen_get_host();
	carmen_moving_objects_map_publish_message(&moving_objects_map_message);
}


static void
publish_a_new_offline_map_if_robot_moved_to_another_block(carmen_point_t *pose, double timestamp, bool force_saving_new_map)
{
	double time_now;

	if (map_file_name)
		return;

	// Mudanca de origin do mapa sendo feita
	carmen_position_t map_origin;
	carmen_grid_mapping_get_map_origin(pose, &map_origin.x, &map_origin.y);

	mapper_save_map = 0;
	update_and_merge_with_mapper_saved_maps = 0;
	if (use_merge_between_maps)
		mapper_change_map_origin_to_another_map_block_with_clones(map_path, current_map_set, &map_origin, mapper_save_map);
	else
		mapper_change_map_origin_to_another_map_block(map_path, current_map_set, &map_origin, mapper_save_map);

	mapper_save_map = 1;
	update_and_merge_with_mapper_saved_maps = 1;

	int map_origin_changed;
	if (use_merge_between_maps)
		map_origin_changed = mapper_change_map_origin_to_another_map_block_with_clones(map_path, updated_offline_map_set, &map_origin, mapper_save_map, force_saving_new_map);
	else
		map_origin_changed = mapper_change_map_origin_to_another_map_block(map_path, updated_offline_map_set, &map_origin, mapper_save_map, force_saving_new_map);
	mapper_save_map = 1;
	update_and_merge_with_mapper_saved_maps = 1;
	// fim Mudanca de origin do mapa sendo feita

	time_now = carmen_get_time();
	static double last_time_changed = 0.0;
	static double begin_global_localization = 0.0;
	if (map_origin_changed || force_saving_new_map || ((time_now - last_time_changed) > time_interval_for_map_change)) // && (updated_offline_map->config.x_origin != x_origin || updated_offline_map->config.y_origin != y_origin))
	{
		carmen_grid_mapping_get_block_map_by_origin(map_path, 'm', *pose, updated_offline_map_set->occupancy_map);
		carmen_grid_mapping_get_block_map_by_origin(map_path, 'u', *pose, updated_offline_map_set->sum_occupancy_map);
		carmen_grid_mapping_get_block_map_by_origin(map_path, 'o', *pose, updated_offline_map_set->count_occupancy_map);

		static carmen_map_t *current_mean_remission_map = NULL;
		static carmen_map_t *current_variance_remission_map = NULL;

		if (use_remission)
		{
			carmen_grid_mapping_get_block_map_by_origin(map_path, 's', *pose, updated_offline_map_set->sum_remission_map);
			carmen_grid_mapping_get_block_map_by_origin(map_path, '2', *pose, updated_offline_map_set->sum_sqr_remission_map);
			carmen_grid_mapping_get_block_map_by_origin(map_path, 'c', *pose, updated_offline_map_set->count_remission_map);

			if (!current_mean_remission_map)
			{
				current_mean_remission_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
				current_mean_remission_map->complete_map = NULL;
				current_mean_remission_map->map = NULL;
				carmen_grid_mapping_initialize_map(current_mean_remission_map, ((double) map_width / map_grid_res), map_grid_res, 'm');

				current_variance_remission_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
				current_variance_remission_map->complete_map = NULL;
				current_variance_remission_map->map = NULL;
				carmen_grid_mapping_initialize_map(current_variance_remission_map, ((double) map_width / map_grid_res), map_grid_res, 'm');
			}
			carmen_prob_models_calc_mean_and_variance_remission_map(current_mean_remission_map, current_variance_remission_map,
					updated_offline_map_set->sum_remission_map, updated_offline_map_set->sum_sqr_remission_map, updated_offline_map_set->count_remission_map);
		}

		if ((last_time_changed == 0.0) || ((time_now - begin_global_localization) < 5.0))
		{
			if (last_time_changed == 0.0)
				begin_global_localization = time_now;
			carmen_to_localize_ackerman_map(updated_offline_map_set->occupancy_map, current_mean_remission_map, current_variance_remission_map,
					&localize_map, &localize_param);
		}
		else
			carmen_to_localize_ackerman_map_only_prob(updated_offline_map_set->occupancy_map, current_mean_remission_map, current_variance_remission_map,
					&localize_map, &localize_param);
		carmen_map_server_publish_localize_map_message(&localize_map);

		strcpy(updated_offline_map_set->occupancy_map->config.origin, "from_param_daemon");
		carmen_map_server_publish_offline_map_message(updated_offline_map_set->occupancy_map, timestamp);
		offline_map_published = 1;

		last_time_changed = time_now;
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


static void
alloc_rddf_global_data(carmen_behavior_selector_path_goals_and_annotations_message *message)
{
	rddf_message = (carmen_rddf_road_profile_message *) calloc (1, sizeof(carmen_rddf_road_profile_message));

	rddf_message->annotations = (int *) calloc (message->number_of_poses, sizeof(int));
	rddf_message->poses = (carmen_robot_and_trailers_traj_point_t *) calloc (message->number_of_poses, sizeof(carmen_robot_and_trailers_traj_point_t));
	rddf_message->number_of_poses = message->number_of_poses;

	rddf_message->poses_back = (carmen_robot_and_trailers_traj_point_t *) calloc (message->number_of_poses_back, sizeof(carmen_robot_and_trailers_traj_point_t));
	rddf_message->number_of_poses_back = message->number_of_poses_back;
}


static void
realloc_rddf_global_data(carmen_behavior_selector_path_goals_and_annotations_message *message)
{
	if (message->number_of_poses != rddf_message->number_of_poses)
	{
		rddf_message->annotations = (int *) realloc (rddf_message->annotations, message->number_of_poses * sizeof(int));
		rddf_message->poses = (carmen_robot_and_trailers_traj_point_t *) realloc (rddf_message->poses, message->number_of_poses * sizeof(carmen_robot_and_trailers_traj_point_t));
	}

	if (message->number_of_poses_back != rddf_message->number_of_poses_back)
		rddf_message->poses_back = (carmen_robot_and_trailers_traj_point_t *) realloc (rddf_message->poses_back, message->number_of_poses_back * sizeof(carmen_robot_and_trailers_traj_point_t));

	rddf_message->number_of_poses = message->number_of_poses;
	rddf_message->number_of_poses_back = message->number_of_poses_back;
}


static void
copy_local_rddf_to_global_rddf(carmen_behavior_selector_path_goals_and_annotations_message *message)
{
	memcpy(rddf_message->annotations, message->annotations, message->number_of_poses * sizeof(int));
	memcpy(rddf_message->poses, message->poses, message->number_of_poses * sizeof(carmen_robot_and_trailers_traj_point_t));
	memcpy(rddf_message->poses_back, message->poses_back, message->number_of_poses_back * sizeof(carmen_robot_and_trailers_traj_point_t));
}


void
mapupdater_mapper_set_robot_pose_into_the_map(carmen_localize_ackerman_globalpos_message *globalpos_message, int UPDATE_CELLS_BELOW_CAR)
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

	current_map_set->occupancy_map->config.x_origin = current_map_set->x_origin;
	current_map_set->occupancy_map->config.y_origin = current_map_set->y_origin;

	if (UPDATE_CELLS_BELOW_CAR)
		carmen_mapper_update_cells_bellow_robot(globalpos_message->globalpos, current_map_set->occupancy_map, 0.0);
//		carmen_prob_models_updade_cells_bellow_robot(globalpos_message->globalpos, &map, 0.0, &car_config);
}


void
include_sensor_data_into_map(int sensor_number, carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	int i, old_point_cloud_index = -1;
	int nearest_global_pos = 0;
	double nearest_time = globalpos_message->timestamp;
	double old_globalpos_timestamp;
	carmen_pose_3D_t old_robot_position;

	if (sensors_data[sensor_number].point_cloud_index == -1)   // No point cloud was received from the sensor
		return;

	for (i = 0; i < NUM_VELODYNE_POINT_CLOUDS; i++)
	{
		double time_difference = fabs(sensors_data[sensor_number].points_timestamp[i] - globalpos_message->timestamp);
		if (time_difference == 0.0)
		{
			old_point_cloud_index = sensors_data[sensor_number].point_cloud_index;
			sensors_data[sensor_number].point_cloud_index = i;
			old_globalpos_timestamp = sensors_data[sensor_number].robot_timestamp[i];
			old_robot_position = sensors_data[sensor_number].robot_pose[i];
			sensors_data[sensor_number].robot_pose[i] = globalpos_message->pose;
			sensors_data[sensor_number].robot_timestamp[i] = globalpos_message->timestamp;
			sensors_data[sensor_number].semi_trailer_data =
			{
					globalpos_message->semi_trailer_engaged,
					globalpos_message->semi_trailer_type,
					semi_trailer_config.semi_trailers[0].d,
					semi_trailer_config.semi_trailers[0].M,
					globalpos_message->trailer_theta[0]
			};

			run_mapper(current_map_set, &sensors_params[sensor_number], &sensors_data[sensor_number], r_matrix_car_to_global);

			sensors_data[sensor_number].robot_pose[i] = old_robot_position;
			sensors_data[sensor_number].robot_timestamp[i] = old_globalpos_timestamp;
			sensors_data[sensor_number].point_cloud_index = old_point_cloud_index;
			break;
		}
		else if (time_difference < nearest_time)
		{
			nearest_global_pos = i;
			nearest_time = time_difference;
		}
	}

	if (i == NUM_VELODYNE_POINT_CLOUDS)
	{
		i = nearest_global_pos;
		old_point_cloud_index = sensors_data[sensor_number].point_cloud_index;
		sensors_data[sensor_number].point_cloud_index = i;
		old_globalpos_timestamp = sensors_data[sensor_number].robot_timestamp[i];
		old_robot_position = sensors_data[sensor_number].robot_pose[i];
		sensors_data[sensor_number].robot_pose[i] = globalpos_message->pose;
		sensors_data[sensor_number].robot_timestamp[i] = globalpos_message->timestamp;
		sensors_data[sensor_number].semi_trailer_data =
		{
				globalpos_message->semi_trailer_engaged,
				globalpos_message->semi_trailer_type,
				semi_trailer_config.semi_trailers[0].d,
				semi_trailer_config.semi_trailers[0].M,
				globalpos_message->trailer_theta[0]
		};

		run_mapper(current_map_set, &sensors_params[sensor_number], &sensors_data[sensor_number], r_matrix_car_to_global);

		sensors_data[sensor_number].robot_pose[i] = old_robot_position;
		sensors_data[sensor_number].robot_timestamp[i] = old_globalpos_timestamp;
		sensors_data[sensor_number].point_cloud_index = old_point_cloud_index;
	}
}


bool
mapupdater(carmen_map_set_t *updated_offline_map_set, carmen_map_set_t *current_map_set)
{
//	carmen_grid_mapping_save_map((char *) "offline.map", updated_offline_map_set->sum_occupancy_map);
//	carmen_grid_mapping_save_map((char *) "current.map", current_map_set->sum_occupancy_map);
//	carmen_grid_mapping_save_map((char *) "current_occ.map", current_map_set->occupancy_map);
//	carmen_grid_mapping_save_map((char *) "offline_occ.map", updated_offline_map_set->occupancy_map);

	static int reverse_occ = 0;
	static int reverse_free = 0;
	int confirme_occ = 0;
	int confirme_free = 0;
	for (int y = 0; y < current_map_set->occupancy_map->config.y_size; y++)
	{
		for (int x = 0; x < current_map_set->occupancy_map->config.x_size; x++)
		{
//			if (x==596 && y==419)
//				num_changes++;
			double occupancy = current_map_set->occupancy_map->map[x][y];
			if ((occupancy >= 0.0) && (occupancy <= 1.0))
			{
				bool offline_occu = updated_offline_map_set->sum_occupancy_map->map[x][y] > sensors_params[VELODYNE].log_odds.log_odds_l0;
				double offline_sum_occu = updated_offline_map_set->sum_occupancy_map->map[x][y];
				double offline_sum_free = updated_offline_map_set->sum_occupancy_map->map[x][y];
				double offline_count = updated_offline_map_set->count_occupancy_map->map[x][y];

				bool current_occu = current_map_set->sum_occupancy_map->map[x][y] > sensors_params[VELODYNE].log_odds.log_odds_l0;
				double current_sum_occu = current_map_set->sum_occupancy_map->map[x][y];
				double current_sum_free = current_map_set->sum_occupancy_map->map[x][y];
				double current_count = current_map_set->count_occupancy_map->map[x][y];

				if (
					((current_occu != offline_occu) &&
					 (current_count > mapupdater_max_count) &&
					 ((current_sum_occu > mapupdater_max_log_odds) ||
					  (current_sum_free < mapupdater_min_log_odds)))
				   )
				{
					updated_offline_map_set->occupancy_map->map[x][y] = current_map_set->occupancy_map->map[x][y];
					updated_offline_map_set->sum_occupancy_map->map[x][y] = current_map_set->sum_occupancy_map->map[x][y];
					updated_offline_map_set->count_occupancy_map->map[x][y] = current_map_set->count_occupancy_map->map[x][y];

					if (use_remission)
					{
						updated_offline_map_set->sum_remission_map->map[x][y] = current_map_set->sum_remission_map->map[x][y];
						updated_offline_map_set->sum_sqr_remission_map->map[x][y] = current_map_set->sum_sqr_remission_map->map[x][y];
						updated_offline_map_set->count_remission_map->map[x][y] = current_map_set->count_remission_map->map[x][y];
					}

					if (offline_occu)
						reverse_occ++;
					else
						reverse_free++;
				}
				else if (
					((current_occu == offline_occu) &&
					 (current_count > mapupdater_max_count) &&
					 ((offline_occu && ((current_count > offline_count) || (current_sum_occu > offline_sum_occu))) ||
					  (!offline_occu && ((current_count > offline_count) || (current_sum_free < offline_sum_free)))))
				   )
				{
					updated_offline_map_set->occupancy_map->map[x][y] = current_map_set->occupancy_map->map[x][y];
					updated_offline_map_set->sum_occupancy_map->map[x][y] = current_map_set->sum_occupancy_map->map[x][y];
					updated_offline_map_set->count_occupancy_map->map[x][y] = current_map_set->count_occupancy_map->map[x][y];

					if (use_remission)
					{
						updated_offline_map_set->sum_remission_map->map[x][y] = current_map_set->sum_remission_map->map[x][y];
						updated_offline_map_set->sum_sqr_remission_map->map[x][y] = current_map_set->sum_sqr_remission_map->map[x][y];
						updated_offline_map_set->count_remission_map->map[x][y] = current_map_set->count_remission_map->map[x][y];
					}

					if (offline_occu)
						confirme_occ++;
					else
						confirme_free++;
				}
			}
		}
	}

	int num_changes = reverse_occ + reverse_free + confirme_occ + confirme_free;
	printf("reverse_occ %d, reverse_free %d, sum_reverse %d, confirme_occ %d, confirme_free %d,  num_changes %d, max %d\n",
			reverse_occ, reverse_free, reverse_occ + reverse_free, confirme_occ, confirme_free, num_changes,
			(int) ((double) (current_map_set->occupancy_map->config.x_size * current_map_set->occupancy_map->config.y_size) * mapupdater_percentage_change_for_update));
	fflush(stdout);
//	if (num_changes > (int) ((double) (current_map_set->occupancy_map->config.x_size * current_map_set->occupancy_map->config.y_size) * mapupdater_percentage_change_for_update))
//		return (true);
//	else
//		return (false);
	if ((reverse_occ + reverse_free) > mapupdater_strenght_dacay)
	{
		reverse_occ = reverse_free = 0;
		return (true);
	}
	else
		return (false);
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	mapupdater_mapper_set_robot_pose_into_the_map(globalpos_message, 1);
	if (globalpos_message->semi_trailer_type != semi_trailer_config.num_semi_trailers)
		read_parameters_semi_trailer(globalpos_message->semi_trailer_type);

	static double previous_timestamp = 0.0;
	if (ok_to_publish)
	{
		include_sensor_data_into_map(VELODYNE, globalpos_message);

		bool force_saving_new_map = false;

		carmen_playback_command(CARMEN_PLAYBACK_COMMAND_STOP, NULL, 0, log_playback_speed);

		if (fabs(globalpos_message->v) > 0.05)
			force_saving_new_map = mapupdater(updated_offline_map_set, current_map_set);

		publish_a_new_offline_map_if_robot_moved_to_another_block(&(globalpos_message->globalpos), globalpos_message->timestamp, force_saving_new_map);

		if (globalpos_message->timestamp > previous_timestamp)	// para evitar que o playback do log reinicie apos o final
			carmen_playback_command(CARMEN_PLAYBACK_COMMAND_PLAY, NULL, 0, log_playback_speed);

		publish_map(globalpos_message->timestamp);

		previous_timestamp = globalpos_message->timestamp;
	}
}


static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	mapper_velodyne_partial_scan(VELODYNE, velodyne_message);
}


//static void
//offline_map_handler(carmen_map_server_offline_map_message *msg)
//{
//	static bool first_time = true;
//	carmen_position_t map_origin;
//
//	if (first_time)
//	{
//		offline_map_available = true;
//		first_time = false;
//	}
//
//	map_origin.x = msg->config.x_origin;
//	map_origin.y = msg->config.y_origin;
//
//	memcpy(offline_map.complete_map, msg->complete_map, msg->config.x_size * msg->config.y_size * sizeof(double));
//	offline_map.config = msg->config;
//
//	if (use_merge_between_maps)
//		mapper_change_map_origin_to_another_map_block_with_clones(map_path, offline_map, &map_origin, mapper_save_map);
//	else
//		mapper_change_map_origin_to_another_map_block(map_path, offline_map, &map_origin, mapper_save_map);
//}


static void
localize_ackerman_initialize_message_handler(carmen_localize_ackerman_initialize_message *msg)
{
	publish_a_new_offline_map_if_robot_moved_to_another_block(msg->mean, msg->timestamp, false);
}


void
path_goals_and_annotations_message_handler(carmen_behavior_selector_path_goals_and_annotations_message *message)
{
	static carmen_point_t pose_in_last_publish = {0.0, 0.0, 0.0};

	if (goal_source_path_planner || rddf_source_tracker)
	{
		if (message->number_of_poses <= 0)
			return;
	}

	else if (message->number_of_poses <= 0 || message->number_of_poses_back <= 0)
			return;

	if (is_first_rddf_message)
	{
		alloc_rddf_global_data(message);
		is_first_rddf_message = 0;
	}
	else
	{
		if (goal_source_path_planner || rddf_source_tracker)
		{
			if ((message->number_of_poses != rddf_message->number_of_poses))
				realloc_rddf_global_data(message);
		}

		else if ((message->number_of_poses != rddf_message->number_of_poses) ||
				(message->number_of_poses_back != rddf_message->number_of_poses_back))
			realloc_rddf_global_data(message);

		copy_local_rddf_to_global_rddf(message);

		double distance_without_lane_map;

		distance_without_lane_map =
				sqrt(pow(pose_g.x - pose_in_last_publish.x, 2) +
						pow(pose_g.y - pose_in_last_publish.y, 2));

		if ((distance_without_lane_map > distance_to_update_lane_map) || (offline_map_published))
		{
//			construct_compressed_lane_map();
//			publish_compressed_lane_map();
			pose_in_last_publish = pose_g;
			offline_map_published = 0;
		}
	}
}


static void
map_request_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	carmen_mapper_map_message map_msg;
	IPC_RETURN_TYPE err;

	if (updated_offline_map_set->occupancy_map->complete_map != NULL)
	{
		IPC_freeByteArray(callData);

		map_msg.config = updated_offline_map_set->occupancy_map->config;
		map_msg.complete_map = updated_offline_map_set->occupancy_map->complete_map;
		map_msg.size = updated_offline_map_set->occupancy_map->config.x_size * updated_offline_map_set->occupancy_map->config.y_size;
		map_msg.host = carmen_get_host();
		map_msg.timestamp = carmen_get_time();

		err = IPC_respondData(msgRef, CARMEN_MAP_SERVER_CURRENT_OFFLINE_MAP_NAME, &map_msg);
		carmen_test_ipc(err, "Could not respond", CARMEN_MAP_SERVER_CURRENT_OFFLINE_MAP_NAME);
	}
}


void
shutdown_module(int sig)
{
	if (sig == SIGINT)
	{
		carmen_ipc_disconnect();

		fprintf(stderr, "\nDisconnecting map_updater.\n");
		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
initialize_structures(void)
{
	localize_map.complete_distance = NULL;
	localize_map.complete_gprob = NULL;
	localize_map.carmen_map.complete_map = NULL;
	localize_map.carmen_map.map = NULL;
	localize_map.complete_prob = NULL;
	localize_map.complete_x_offset = NULL;
	localize_map.complete_y_offset = NULL;
	localize_map.distance = NULL;
	localize_map.gprob = NULL;
	localize_map.prob = NULL;
	localize_map.x_offset = NULL;
	localize_map.y_offset = NULL;

//	current_road_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
//	current_road_map->config.x_origin = current_road_map->config.y_origin = 0.0001;
//	current_road_map->complete_map = NULL;
//	current_road_map->map = NULL;
//
//	carmen_grid_mapping_initialize_map(current_road_map, ((double) map_width / map_grid_res), map_grid_res, 'r');
}


void
carmen_mapupdater_initilize()
{
	memset(base_ackerman_odometry_vector, 0, BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE * sizeof(carmen_base_ackerman_odometry_message));
	memset(fused_odometry_vector, 0, FUSED_ODOMETRY_VECTOR_SIZE * sizeof(carmen_fused_odometry_message));
	globalpos_history = (carmen_localize_ackerman_globalpos_message *) calloc(GLOBAL_POS_QUEUE_SIZE, sizeof(carmen_localize_ackerman_globalpos_message));

	mapper_save_map = 1;
	use_remission = 1;
	create_map_sum_and_count = 1;
}


void
carmen_mapupdater_define_messages()
{
	carmen_localize_ackerman_define_globalpos_messages();
}


static void
carmen_mapupdater_subscribe_ipc_messages()
{
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) carmen_localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);
//	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) offline_map_handler, CARMEN_SUBSCRIBE_LATEST);

	IPC_RETURN_TYPE err = IPC_subscribe(CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME, map_request_handler, NULL);
	carmen_test_ipc(err, "Could not subscribe", CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME);

	IPC_setMsgQueueLength(CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME, 100);

	carmen_localize_ackerman_subscribe_initialize_message(NULL, (carmen_handler_t) localize_ackerman_initialize_message_handler, CARMEN_SUBSCRIBE_LATEST);

//	carmen_behavior_selector_subscribe_path_goals_and_annotations_message(NULL, (carmen_handler_t) (path_goals_and_annotations_message_handler), CARMEN_SUBSCRIBE_LATEST);
}


void 
carmen_mapupdater_read_parameters(int argc, char **argv)
{
	carmen_param_allow_unfound_variables(0);

	carmen_param_t basic_param_list[] = 
	{
		{(char *) "mapupdater", (char *) "percentage_change_for_update", CARMEN_PARAM_DOUBLE, &mapupdater_percentage_change_for_update, 1, NULL},
		{(char *) "mapupdater", (char *) "max_log_odds", CARMEN_PARAM_DOUBLE, &mapupdater_max_log_odds, 1, NULL},
		{(char *) "mapupdater", (char *) "min_log_odds", CARMEN_PARAM_DOUBLE, &mapupdater_min_log_odds, 1, NULL},
		{(char *) "mapupdater", (char *) "max_count", CARMEN_PARAM_DOUBLE, &mapupdater_max_count, 1, NULL},
		{(char *) "mapupdater", (char *) "strenght_dacay", CARMEN_PARAM_DOUBLE, &mapupdater_strenght_dacay, 1, NULL},
	};

	carmen_param_install_params(argc, argv, basic_param_list, sizeof(basic_param_list) / sizeof(basic_param_list[0]));

	carmen_param_allow_unfound_variables(1);

	carmen_param_t param_optional_list[] =
	{
	};

	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));

	decay_to_offline_map = 0;

	sensors_params[VELODYNE].range_max = 30.0;
}


static void
read_localize_parameters(int argc, char **argv)
{
	double integrate_angle_deg;

	integrate_angle_deg = 1.0;

	carmen_param_t param_list[] =
	{
		{(char *) "robot", (char *) "frontlaser_offset", CARMEN_PARAM_DOUBLE, &localize_param.front_laser_offset, 0, NULL},
		{(char *) "robot", (char *) "rearlaser_offset", CARMEN_PARAM_DOUBLE, &localize_param.rear_laser_offset, 0, NULL},
		{(char *) "localize", (char *) "use_rear_laser", CARMEN_PARAM_ONOFF, &localize_param.use_rear_laser, 0, NULL},
		{(char *) "localize", (char *) "num_particles", CARMEN_PARAM_INT, &localize_param.num_particles, 0, NULL},
		{(char *) "localize", (char *) "laser_max_range", CARMEN_PARAM_DOUBLE, &localize_param.max_range, 1, NULL},
		{(char *) "localize", (char *) "min_wall_prob", CARMEN_PARAM_DOUBLE, &localize_param.min_wall_prob, 0, NULL},
		{(char *) "localize", (char *) "outlier_fraction", CARMEN_PARAM_DOUBLE, &localize_param.outlier_fraction, 0, NULL},
		{(char *) "localize", (char *) "update_distance", CARMEN_PARAM_DOUBLE, &localize_param.update_distance, 0, NULL},
		{(char *) "localize", (char *) "integrate_angle_deg", CARMEN_PARAM_DOUBLE, &integrate_angle_deg, 0, NULL},
		{(char *) "localize", (char *) "do_scanmatching", CARMEN_PARAM_ONOFF, &localize_param.do_scanmatching, 1, NULL},
		{(char *) "localize", (char *) "constrain_to_map", CARMEN_PARAM_ONOFF, &localize_param.constrain_to_map, 1, NULL},
		{(char *) "localize", (char *) "occupied_prob", CARMEN_PARAM_DOUBLE, &localize_param.occupied_prob, 0, NULL},
		{(char *) "localize", (char *) "global_evidence_weight", CARMEN_PARAM_DOUBLE, &localize_param.global_evidence_weight, 0, NULL},
		{(char *) "localize", (char *) "global_distance_threshold", CARMEN_PARAM_DOUBLE, &localize_param.global_distance_threshold, 1, NULL},
		{(char *) "localize", (char *) "global_test_samples", CARMEN_PARAM_INT, &localize_param.global_test_samples, 1, NULL},
		{(char *) "localize", (char *) "use_sensor", CARMEN_PARAM_ONOFF, &localize_param.use_sensor, 0, NULL},
		{(char *) "localize", (char *) "use_log_odds", CARMEN_PARAM_ONOFF, &localize_param.use_log_odds, 0, NULL},
		{(char *) "localize", (char *) "lmap_std", CARMEN_PARAM_DOUBLE, &localize_param.lmap_std, 0, NULL},
		{(char *) "localize", (char *) "tracking_beam_minlikelihood", CARMEN_PARAM_DOUBLE, &localize_param.tracking_beam_minlikelihood, 0, NULL},
		{(char *) "localize", (char *) "tracking_beam_maxlikelihood", CARMEN_PARAM_DOUBLE, &localize_param.tracking_beam_maxlikelihood, 0, NULL},
		{(char *) "localize", (char *) "global_lmap_std", CARMEN_PARAM_DOUBLE, &localize_param.global_lmap_std, 0, NULL},
		{(char *) "localize", (char *) "global_beam_minlikelihood", CARMEN_PARAM_DOUBLE, &localize_param.global_beam_minlikelihood, 0, NULL},
		{(char *) "localize", (char *) "global_beam_maxlikelihood", CARMEN_PARAM_DOUBLE, &localize_param.global_beam_maxlikelihood, 0, NULL},
		{(char*)"robot",  (char*)"distance_between_front_car_and_front_wheels", 	CARMEN_PARAM_DOUBLE, &(car_config.distance_between_front_car_and_front_wheels), 1, NULL},
		{(char*)"robot",  (char*)"distance_between_front_and_rear_axles",		CARMEN_PARAM_DOUBLE, &(car_config.distance_between_front_and_rear_axles), 1, NULL},
		{(char*)"robot",  (char*)"distance_between_rear_car_and_rear_wheels",		CARMEN_PARAM_DOUBLE, &(car_config.distance_between_rear_car_and_rear_wheels), 1, NULL},
		{(char*)"robot",  (char*)"distance_between_rear_wheels",			CARMEN_PARAM_DOUBLE, &(car_config.distance_between_rear_wheels), 1, NULL},
		{(char *) "behavior_selector",   (char *) "goal_source_path_planner", 		CARMEN_PARAM_ONOFF,  &goal_source_path_planner, 0, NULL},
		{(char *) "rddf",   (char *) "source_tracker", 		CARMEN_PARAM_ONOFF,  &rddf_source_tracker, 0, NULL}
	};


	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	localize_param.integrate_angle = carmen_degrees_to_radians(integrate_angle_deg);
}


static void
read_map_server_parameters(int argc, char **argv)
{
	double robot_width, obstacle_avoider_obstacles_safe_distance;

	carmen_param_t param_list[] =
	{
		{(char *) "map_server", (char *) "initial_waiting_time", 					CARMEN_PARAM_DOUBLE, &initial_waiting_time, 0, NULL},
		{(char *) "map_server", (char *) "map_grid_res", 							CARMEN_PARAM_DOUBLE, &map_grid_res, 0, NULL},
		{(char *) "map_server", (char *) "map_width", 							CARMEN_PARAM_DOUBLE, &map_width, 0, NULL},
		{(char *) "map_server", (char *) "map_height", 							CARMEN_PARAM_DOUBLE, &map_height, 0, NULL},
		{(char *) "map_server", (char *) "time_interval_for_map_change", 			CARMEN_PARAM_DOUBLE, &time_interval_for_map_change, 0, NULL},
		{(char *) "map_server", (char *) "publish_google_map", 					CARMEN_PARAM_ONOFF, &publish_google_map, 1, NULL},
		{(char *) "model_predictive_planner", (char *) "obstacles_safe_distance",	CARMEN_PARAM_DOUBLE, &lane_width, 1, NULL},
		{(char *) "obstacle_avoider", (char *) "obstacles_safe_distance",			CARMEN_PARAM_DOUBLE, &obstacle_avoider_obstacles_safe_distance, 1, NULL},
		{(char *) "robot", (char *) "width",										CARMEN_PARAM_DOUBLE, &robot_width, 1, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	// @@@ Alberto: Mudancca abaixo para acomodar as coisas novas feitas pelo Renan. lane_width vai ficar igual aa metade largura do carro mais a distancia de segurancca do MPP
	lane_width += (robot_width + 2.0 * obstacle_avoider_obstacles_safe_distance) / 2.0;
}


static void
read_optional_map_server_parameters(int argc, char **argv)
{
	//default parameters
	block_map = 0;

	carmen_param_allow_unfound_variables(1);

	carmen_param_t optional_param_list[] =
	{
		{(char *) "commandline", (char *) "block_map", CARMEN_PARAM_ONOFF, &block_map, 0, NULL},
		{(char *) "commandline", (char *) "map_x", CARMEN_PARAM_INT, &initial_map_x, 0, NULL},
		{(char *) "commandline", (char *) "map_y", CARMEN_PARAM_INT, &initial_map_y, 0, NULL},
		{(char *) "commandline", (char *) "map_path", CARMEN_PARAM_STRING, &map_path, 0, NULL},
		{(char *) "commandline", (char *) "map", CARMEN_PARAM_STRING, &map_file_name, 0, NULL},
		{(char *) "commandline", (char *) "publish_grid_mapping_map_at_startup", CARMEN_PARAM_ONOFF, &publish_grid_mapping_map_at_startup, 0, NULL},
		{(char *) "commandline", (char *) "lanemap_incoming_message_type", CARMEN_PARAM_INT, &lanemap_incoming_message_type, 0, NULL} // 0 - road_profile (RDDF), 1 - astar, 2 - spline
	};

	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));
}


static void
read_all_map_server_parameters(int argc, char **argv)
{
	read_map_server_parameters(argc, argv);
	read_localize_parameters(argc, argv);
	read_optional_map_server_parameters(argc, argv);
}


void
get_first_map()
{
	if (map_file_name != NULL)
	{
		int no_valid_map_on_file = carmen_map_read_gridmap_chunk(map_file_name, updated_offline_map_set->occupancy_map) != 0;
		if (no_valid_map_on_file)
			printf("mapupdater: could not read offline map from file named: %s\n", map_file_name);
	}
	else
	{
		carmen_point_t pose;
		pose.x = initial_map_x;
		pose.y = initial_map_y;
		if (block_map)
			carmen_grid_mapping_get_block_map_by_origin(map_path, 'm', pose, updated_offline_map_set->occupancy_map);
	}

	if (updated_offline_map_set->occupancy_map->complete_map != NULL)
	{
		double timestamp = carmen_get_time();
		carmen_to_localize_ackerman_map(updated_offline_map_set->occupancy_map, NULL, NULL, &localize_map, &localize_param);
		carmen_map_server_publish_offline_map_message(updated_offline_map_set->occupancy_map, timestamp);
		carmen_map_server_publish_localize_map_message(&localize_map);
		if (publish_grid_mapping_map_at_startup)
			carmen_mapper_publish_map_message(updated_offline_map_set->occupancy_map, timestamp);
	}
	else
		printf("mapupdater: could not get an offline map at startup!\n");
}

// Sempre que mudar o suficente um mapa, publica o atual e os napas necessarios para localização.

int 
main(int argc, char ** argv)
{
	usleep(initial_waiting_time * 1e6);	// Se nao esperar aqui, os mapas iniciais nao sao recebidos pelo localize_ackerman...
	signal(SIGINT, shutdown_module);
	carmen_randomize(&argc, &argv);
	carmen_ipc_initialize(argc, argv);

	carmen_mapper_read_parameters(argc, argv, &map_config, &car_config);
	carmen_mapper_get_highest_sensor();
	carmen_mapper_initialize_transforms();

	carmen_mapupdater_read_parameters(argc, argv);

	current_map_set = mapper_initialize(&map_config, car_config, use_merge_between_maps);
	updated_offline_map_set = get_a_map_set(map_config, use_merge_between_maps, create_map_sum_and_count, use_remission);

	// Map server
	read_all_map_server_parameters(argc, argv);
	carmen_grid_mapping_init_parameters(map_grid_res, map_width);
	initialize_structures();
	get_first_map();

	carmen_mapupdater_initilize();
	carmen_mapupdater_define_messages();
	carmen_mapupdater_subscribe_ipc_messages();

	carmen_warn("Map Updater initialized.\n");

	carmen_ipc_dispatch();

	return (0);
}

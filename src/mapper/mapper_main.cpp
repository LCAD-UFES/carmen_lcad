#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <prob_interface.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/laser_ldmrs_interface.h>
#include <carmen/laser_ldmrs_utils.h>
#include <carmen/rotation_geometry.h>
#include <carmen/mapper_interface.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/stereo_velodyne_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/stereo_mapping_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_util.h>
#include <carmen/ultrasonic_filter_interface.h>
#include <carmen/parking_assistant_interface.h>
#include <omp.h>
#include "mapper.h"

#include "message_interpolation.cpp"


carmen_map_t offline_map;
// TODO: @@@ Alberto: essa variavel eh definida como externa dentro da lib do mapper. Corrigir!
carmen_localize_ackerman_globalpos_message *globalpos_history;
int last_globalpos;

extern int visual_odometry_is_global_pos;
static int parking_assistant_found_safe_space = 0;

MessageInterpolation<carmen_localize_ackerman_globalpos_message, carmen_ultrasonic_sonar_sensor_message> interpolator(1);

extern carmen_pose_3D_t ultrasonic_sensor_r1_g;
extern carmen_pose_3D_t ultrasonic_sensor_r2_g;
extern carmen_pose_3D_t ultrasonic_sensor_l1_g;
extern carmen_pose_3D_t ultrasonic_sensor_l2_g;


/**
 * Model params
 */

double safe_range_above_sensors;
double robot_wheel_radius;

int use_simulator_pose = 0;

double highest_sensor = 0.0;

extern int merge_with_offline_map;
int build_snapshot_map;
extern int update_cells_below_car;
int update_and_merge_with_mapper_saved_maps;
int update_and_merge_with_snapshot_map;
int decay_to_offline_map;
int create_map_sum_and_count;
int use_remission;
extern double mapper_velodyne_range_max;
extern double mapper_range_max_factor;
extern double mapper_unsafe_height_above_ground;     // Points above and bellow this value will not be used by the mapper
extern int mapper_save_map;
extern double rays_threshold_to_merge_between_maps;
extern bool use_merge_between_maps;

extern carmen_pose_3D_t sensor_board_1_pose;
extern carmen_pose_3D_t front_bullbar_pose;

extern sensor_parameters_t *sensors_params;
extern sensor_parameters_t ultrasonic_sensor_params;
extern sensor_data_t *sensors_data;

extern int number_of_sensors;

#define MAX_NUMBER_OF_LIDARS 16   // 15 is the Maximum number of carmen_velodyne_variable_scan_message defined, so is the maximun number of lidars

// const int number_of_lidars = 12; //lidar parameters
const int first_lidar_number = 10;
extern carmen_semi_trailer_config_t semi_trailer_config;
extern carmen_pose_3D_t velodyne_pose;
extern carmen_pose_3D_t laser_ldmrs_pose;
extern carmen_lidar_config lidar_config[MAX_NUMBER_OF_LIDARS];

char *map_path;

extern int publish_moving_objects_raw_map;

carmen_rddf_annotation_message last_rddf_annotation_message;
int robot_near_strong_slow_down_annotation = 0;

bool offline_map_available = false;
int ok_to_publish = 0;
int number_of_threads = 1;

int camera3_ready = 0;
int camera7_ready = 0;

int lidar_ready = 0;

/******variables for neural_mapper dataset*****/
int use_neural_mapper = 0;
int generate_neural_mapper_dataset = 0;
int neural_mapper_max_distance_meters = 0;
int neural_mapper_data_pace = 0;
extern int neural_map_num_clouds;
/**********************/

rotation_matrix *r_matrix_car_to_global = NULL;

extern int use_truepos;

double time_secs_between_map_save = 0.0;

extern int mapping_mode;

extern carmen_mapper_virtual_laser_message virtual_laser_message;

extern carmen_moving_objects_point_clouds_message *moving_objects_message;

extern carmen_mapper_virtual_scan_message virtual_scan_message;

extern char *calibration_file;
extern char *save_calibration_file;

int use_unity_simulator = 0;

extern carmen_map_t occupancy_map;

extern int publish_diff_map;
extern double publish_diff_map_interval;

extern int g_argc;
extern char **g_argv;

extern tf::Transformer tf_transformer;

carmen_mapper_probability_of_each_ray_of_lidar_hit_obstacle_message probability_of_each_ray_msg_array[16];
extern carmen_robot_ackerman_config_t car_config; // TODO essa variável deveria mesmo ser extern???????
//#define OBSTACLE_PROBABILY_MESSAGE


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
					semi_trailer_config.d,
					semi_trailer_config.M,
					globalpos_message->beta
			};

			if (use_merge_between_maps)
				run_mapper_with_remision_threshold(&sensors_params[sensor_number], &sensors_data[sensor_number], r_matrix_car_to_global, rays_threshold_to_merge_between_maps);
			else
				run_mapper(&sensors_params[sensor_number], &sensors_data[sensor_number], r_matrix_car_to_global);

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
				semi_trailer_config.d,
				semi_trailer_config.M,
				globalpos_message->beta
		};

		if (use_merge_between_maps)
			run_mapper_with_remision_threshold(&sensors_params[sensor_number], &sensors_data[sensor_number], r_matrix_car_to_global, rays_threshold_to_merge_between_maps);
		else
			run_mapper(&sensors_params[sensor_number], &sensors_data[sensor_number], r_matrix_car_to_global);

		sensors_data[sensor_number].robot_pose[i] = old_robot_position;
		sensors_data[sensor_number].robot_timestamp[i] = old_globalpos_timestamp;
		sensors_data[sensor_number].point_cloud_index = old_point_cloud_index;
	}
}


void
include_lidars_data_into_map(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	for(int i = 0; i < MAX_NUMBER_OF_LIDARS; i++)
	{
		if (sensors_params[i + 10].alive)  // Lidars start from 10 in the sensors_params vector
		{
			include_sensor_data_into_map(i + 10, globalpos_message);
		}
	}
}


void
free_virtual_scan_message()
{
	if (virtual_scan_message.num_sensors != 0)
	{
		for (int i = 0; i < virtual_scan_message.num_sensors; i++)
		{
			free(virtual_scan_message.virtual_scan_sensor[i].points);
			free(virtual_scan_message.virtual_scan_sensor);
			virtual_scan_message.virtual_scan_sensor = NULL;
			virtual_scan_message.num_sensors = 0;
		}
	}
}


// static void
// read_parameters_semi_trailer(int semi_trailer_type)
// {
// 	semi_trailer_config.type = semi_trailer_type;
// 	if (semi_trailer_type == 0)
// 		return;

// 	char semi_trailer_string[2048];

// 	sprintf(semi_trailer_string, "%s%d", "semi_trailer", semi_trailer_config.type);

// 	char *semi_trailer_poly_file;

// 	carmen_param_t semi_trailer_param_list[] = {
// 		{semi_trailer_string, (char *) "d",								 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.d),							   	  0, NULL},
// 		{semi_trailer_string, (char *) "M",								 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.M),							   	  0, NULL},
// 		{semi_trailer_string, (char *) "width",							 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.width),							  0, NULL},
// 		{semi_trailer_string, (char *) "distance_between_axle_and_front", 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.distance_between_axle_and_front), 0, NULL},
// 		{semi_trailer_string, (char *) "distance_between_axle_and_back",	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.distance_between_axle_and_back),  0, NULL},
// 		{semi_trailer_string, (char *) "max_beta",						 	CARMEN_PARAM_DOUBLE, &(semi_trailer_config.max_beta),						  0, NULL},
// 		{semi_trailer_string, (char *) "polygon_file",					 	CARMEN_PARAM_STRING, &(semi_trailer_poly_file), 							  	0, NULL}
// 	};
// 	carmen_param_install_params(g_argc, g_argv, semi_trailer_param_list, sizeof(semi_trailer_param_list)/sizeof(semi_trailer_param_list[0]));

// 	semi_trailer_config.max_beta = carmen_degrees_to_radians(semi_trailer_config.max_beta);
// }

//----------------------------------------------------------RANIK-------------------------------------------------------------------

// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO checar essa função
void
get_occupancy_log_odds_of_each_ray_target(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data, int scan_index)
{
	int thread_id = omp_get_thread_num();
	int point_cloud_index = sensor_data->point_cloud_index;
	spherical_point_cloud v_zt = sensor_data->points[point_cloud_index];
	int N = v_zt.num_points / sensor_params->vertical_resolution;
	double v = sensor_data->robot_velocity[point_cloud_index].x;
	double phi = sensor_data->robot_phi[point_cloud_index];
	double dt = sensor_params->time_spent_by_each_scan;
	double dt1 = sensor_data->points_timestamp[point_cloud_index] - sensor_data->robot_timestamp[point_cloud_index] - (double) N * dt;
	carmen_pose_3D_t robot_interpolated_position = sensor_data->robot_pose[point_cloud_index];
	carmen_pose_3D_t robot_pose = sensor_data->robot_pose[point_cloud_index];
	double dt2 = dt * scan_index / sensor_params->vertical_resolution;
	robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(robot_pose,
			dt1 + dt2, v, phi, car_config.distance_between_front_and_rear_axles);

	r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

	change_sensor_rear_range_max(sensor_params, v_zt.sphere_points[scan_index].horizontal_angle);

	carmen_prob_models_compute_relevant_map_coordinates_with_remission_check(sensor_data, sensor_params, scan_index, robot_interpolated_position.position,
			sensor_params->sensor_support_pose, r_matrix_car_to_global, sensor_params->support_to_car_matrix,
			robot_wheel_radius, offline_map.config.x_origin, offline_map.config.y_origin, &car_config, robot_near_strong_slow_down_annotation, thread_id, use_remission);

	carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(sensor_data, sensor_params, scan_index, highest_sensor, safe_range_above_sensors,
			robot_near_strong_slow_down_annotation, thread_id);
			// Updates: sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][ray_index] : 0 <= ray_index < 32
}

void
carmen_mapper_fill_probability_of_each_ray_of_lidar_hit_obstacle_message(sensor_parameters_t *sensor_params, sensor_data_t *sensor_data,
		carmen_mapper_probability_of_each_ray_of_lidar_hit_obstacle_message *prob_msg)
{
	int cloud_index = sensor_data->point_cloud_index;
	int vertical_resolution = sensor_params->vertical_resolution;
	int number_of_laser_shots = sensor_data->points[cloud_index].num_points / vertical_resolution;
	int thread_id = omp_get_thread_num();

	if (prob_msg->scan == NULL)
		carmen_mapper_alloc_probability_of_each_ray_of_lidar_hit_obstacle_message(prob_msg, vertical_resolution, number_of_laser_shots);  // TODO realocar caso mude o tamanho

	prob_msg->timestamp = sensor_data->points_timestamp[cloud_index]; // timestamp precisa ser igual ao da mensagem variable scan

	for (int j = 0; j < number_of_laser_shots; j++)
	{
		int scan_index = j * vertical_resolution;
		double horizontal_angle = - sensor_data->points[cloud_index].sphere_points[scan_index].horizontal_angle;

		get_occupancy_log_odds_of_each_ray_target(sensor_params, sensor_data, scan_index);

		for (int i = 1; i < vertical_resolution; i++)
		{
			double vertical_angle = sensor_data->points[cloud_index].sphere_points[scan_index + i].vertical_angle;
			double range = sensor_data->points[cloud_index].sphere_points[scan_index + i].length;

			double log_odds = sensor_data->occupancy_log_odds_of_each_ray_target[thread_id][i];
			double prob = carmen_prob_models_log_odds_to_probabilistic(log_odds);
			//prob contem a probabilidade de um raio ter atingido um obstáculo ou não, no mapper tudo com prob>0.5 atingiu um obstáculo
			prob = prob < 0.0 ? 0.0 : prob;
			// Retirar a parte inteira (caso exista), e multiplicar a parte decimal por 50.000 para caber em um short. Quando esse valor for usado em outros módulos, é necessário dividir por 50.000
			prob_msg->scan[j].probability[i] = (prob - floor(prob)) * 50000;
//			prob_msg->scan[j]->probability[i] = prob;

		}
	}
}


void
fill_and_publish_probabilities_message()
{
	for(int i = 0; i < MAX_NUMBER_OF_LIDARS; i++)
	{
		if (sensors_params[i + 10].alive)  // Lidars start from 10 in the sensors_params vector
		{
			carmen_mapper_fill_probability_of_each_ray_of_lidar_hit_obstacle_message(&sensors_params[i + 10], &sensors_data[i + 10], &probability_of_each_ray_msg_array[i]);
			carmen_mapper_publish_probability_of_each_ray_of_lidar_hit_obstacle_message(&probability_of_each_ray_msg_array[i], i);
		}
	}
}


//-----------------------------------------------------------------------------------------------------------------------------


///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
do_publish_diff_map(carmen_map_t *base_map, carmen_map_t *new_map, double timestamp)
{
	if (base_map == NULL || new_map == NULL ||
		base_map->config.x_origin   != new_map->config.x_origin   || base_map->config.y_origin != new_map->config.y_origin ||
		base_map->config.x_size     != new_map->config.x_size     || base_map->config.y_size   != new_map->config.y_size   ||
		base_map->config.resolution != new_map->config.resolution)
		return;

	static double time_of_last_publish = 0.0;
	if (publish_diff_map && ((timestamp - time_of_last_publish) < publish_diff_map_interval))
		return;
	time_of_last_publish = timestamp;

	carmen_map_config_t config = base_map->config;
	int size = 0;
	int number_of_cells = base_map->config.x_size * base_map->config.y_size;
	short int *coord_x = (short int *) malloc(number_of_cells * sizeof(short int));
	short int *coord_y = (short int *) malloc(number_of_cells * sizeof(short int));
	unsigned char *value = (unsigned char *) malloc(number_of_cells * sizeof(unsigned char));
	carmen_test_alloc(coord_x);
	carmen_test_alloc(coord_y);
	carmen_test_alloc(value);

	for (int i = 0; i < number_of_cells; i++)
	{
		int new_value = int(new_map->complete_map[i] * 255.0);
		if (new_value != int(base_map->complete_map[i] * 255.0))
		{
			coord_x[size] = i / config.y_size;
			coord_y[size] = i % config.y_size;
			value[size] = (unsigned char) new_value;
			size++;
		}
	}

	carmen_mapper_publish_diff_map_message(coord_x, coord_y, value, size, config, timestamp);

	free(coord_x);
	free(coord_y);
	free(value);
}


static void
publish_map(double timestamp)
{
//	mapper_publish_map(timestamp);
	if (build_snapshot_map)
	{
		memcpy(occupancy_map.complete_map, offline_map.complete_map, offline_map.config.x_size *  offline_map.config.y_size * sizeof(double));
		// memset(map.complete_map, 0, offline_map.config.x_size *  offline_map.config.y_size * sizeof(double));         // Uncomment to see the snapshot_map on viewer 3D, on carmen-ford-scape.ini turn on mapper_build_snapshot_map an turn off mapper_decay_to_offline_map
		run_snapshot_mapper();
	}

	add_virtual_laser_points(&occupancy_map, &virtual_laser_message);

	add_moving_objects(&occupancy_map, moving_objects_message);

	// Publica o mapa compactado aoenas com as celulas com probabilidade igual ou maior que 0.5
	carmen_compact_map_t cmap;
	carmen_prob_models_create_compact_map_with_cells_larger_than_value(&cmap, &occupancy_map, 0.5);
	carmen_mapper_publish_compact_map_message(&cmap, timestamp);
	carmen_prob_models_free_compact_map(&cmap);

	// Publica o mapa nao compactado
	carmen_mapper_publish_map_message(&occupancy_map, timestamp);
//	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, timestamp);
//	printf("n = %d\n", virtual_laser_message.num_positions);
	do_publish_diff_map(&offline_map, &occupancy_map, timestamp);
}


void
publish_virtual_scan(double timestamp)
{
	carmen_mapper_publish_virtual_scan_message(&virtual_scan_message, timestamp);
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
	if (visual_odometry_is_global_pos)
		interpolator.AddMessageToInterpolationList(globalpos_message);
	else
		mapper_set_robot_pose_into_the_map(globalpos_message, update_cells_below_car);

	// printf("beta %lf\n", globalpos_message->beta);
	// Map annotations handling
	double distance_to_nearest_annotation = 1000.0;
	int index_of_nearest_annotation = 0;

	for (int i = 0; i < last_rddf_annotation_message.num_annotations; i++)
	{
		double distance_to_annotation = DIST2D(last_rddf_annotation_message.annotations[i].annotation_point, globalpos_history[last_globalpos].pose.position);
		if ((distance_to_annotation < distance_to_nearest_annotation) &&
			((last_rddf_annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_BUMP) ||
			(last_rddf_annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_BARRIER) ||
			((last_rddf_annotation_message.annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			(last_rddf_annotation_message.annotations[i].annotation_code <= RDDF_ANNOTATION_CODE_SPEED_LIMIT_20))))
		{
			distance_to_nearest_annotation = distance_to_annotation;
			index_of_nearest_annotation = i;
		}
	}
	if (((distance_to_nearest_annotation < 50.0) &&
		carmen_rddf_play_annotation_is_forward(globalpos_message->globalpos,
		last_rddf_annotation_message.annotations[index_of_nearest_annotation].annotation_point)) ||
		(distance_to_nearest_annotation < 8.0))
	{
		robot_near_strong_slow_down_annotation = 1;
	}
	else
	{
		robot_near_strong_slow_down_annotation = 0;
	}

	if (globalpos_message->semi_trailer_type != semi_trailer_config.type)
		read_parameters_semi_trailer(globalpos_message->semi_trailer_type);

	if (ok_to_publish)
	{
		if (decay_to_offline_map)
			map_decay_to_offline_map(&occupancy_map);

		free_virtual_scan_message();

		// A ordem eh importante
		if (sensors_params[VELODYNE].alive)
			include_sensor_data_into_map(VELODYNE, globalpos_message);
		
		include_lidars_data_into_map(globalpos_message);

		if (sensors_params[LASER_LDMRS].alive && !robot_near_strong_slow_down_annotation)
			include_sensor_data_into_map(LASER_LDMRS, globalpos_message);
		if (sensors_params[3].alive && camera3_ready)	// camera 3
			include_sensor_data_into_map(3, globalpos_message);
		if (sensors_params[7].alive && camera7_ready)	// camera 3
			include_sensor_data_into_map(7, globalpos_message);

		camera3_ready = 0;
		camera7_ready = 0;

		publish_map(globalpos_message->timestamp);
		publish_virtual_scan(globalpos_message->timestamp);

		#ifdef OBSTACLE_PROBABILY_MESSAGE
		fill_and_publish_probabilities_message();
//		    carmen_mapper_fill_probability_of_each_ray_of_lidar_hit_obstacle_message(&sensors_params[0 + 10], &sensors_data[0 + 10], &probability_of_each_ray_msg_0);
//			carmen_mapper_publish_probability_of_each_ray_of_lidar_hit_obstacle_message(&probability_of_each_ray_msg_0, 0);
		#endif
	}

	if (update_and_merge_with_mapper_saved_maps && time_secs_between_map_save > 0.0 && mapper_save_map)
		mapper_periodically_save_current_map(globalpos_message->timestamp);
}


static void
true_pos_message_handler(carmen_simulator_ackerman_truepos_message *pose)
{
	if (offline_map_available && !ok_to_publish)
	{
		map_decay_to_offline_map(get_the_map());
		publish_map(pose->timestamp);

		// O codigo abaixo publica mensagens de Velodyne fake ateh que o mecanismo padrao de publicacao de mapas comece a funcionar
		// Quando comeca, ok_to_publish se torna 1. ok_to_publish indica quando esta ok para a publicacao de mapas.
		carmen_velodyne_partial_scan_message fake_velodyne_message;
		fake_velodyne_message.number_of_32_laser_shots = 1;

		carmen_velodyne_32_laser_shot fake_shot;
		fake_shot.angle = 0.0;

		for (int i = 0; i < 32; i++)
		{
			fake_shot.distance[i] = 1; // Bem curtinho, mas nao zero. Zero eh tratado como infinito e gera escrita no mapa por ray casting.
			fake_shot.intensity[i] = 0;
		}

		fake_velodyne_message.partial_scan = &fake_shot;
		fake_velodyne_message.timestamp = pose->timestamp;
		fake_velodyne_message.host = carmen_get_host();
		mapper_velodyne_partial_scan(VELODYNE, &fake_velodyne_message);
	}
}

void
setup_message(carmen_velodyne_variable_scan_message &msg, int number_of_shots, int shot_size)
{
	msg.partial_scan = (carmen_velodyne_shot *) malloc ((number_of_shots + 1) * sizeof(carmen_velodyne_shot));
	
    for (int i = 0 ; i <= number_of_shots; i++)
	{
		msg.partial_scan[i].shot_size = shot_size;
		msg.partial_scan[i].distance  = (unsigned int*) malloc (shot_size * sizeof(unsigned int));
		msg.partial_scan[i].intensity = (unsigned short*)  malloc (shot_size * sizeof(unsigned short));
	}
	msg.host = carmen_get_host();
}

static void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	mapper_velodyne_partial_scan(VELODYNE, velodyne_message);

	// static bool first_time = true;
	// static carmen_velodyne_variable_scan_message msg;

	// if (first_time)
	// {
	// 	setup_message(msg, 4000, 32);
	// 	first_time = false;
	// }
	// msg.number_of_shots = velodyne_message->number_of_32_laser_shots;
	
	// for(int i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	// {
	// 	msg.partial_scan[i].distance = velodyne_message->partial_scan[i].distance;
	// 	msg.partial_scan[i].intensity = velodyne_message->partial_scan[i].intensity;
	// 	msg.partial_scan[i].angle = velodyne_message->partial_scan[i].angle;
	// }
	// msg.timestamp = velodyne_message->timestamp;

	// update_data_params_with_lidar_data(10, &msg);
}

static void
laser_ldrms_message_handler(carmen_laser_ldmrs_message *laser) // old handler not used anymore
{
	carmen_velodyne_partial_scan_message partial_scan_message = carmen_laser_ldmrs_convert_laser_scan_to_partial_velodyne_message(laser, laser->timestamp);

	if (partial_scan_message.number_of_32_laser_shots > 0)
	{
		mapper_velodyne_partial_scan(LASER_LDMRS, &partial_scan_message);
		free(partial_scan_message.partial_scan);
	}
}


static void
laser_ldrms_new_message_handler(carmen_laser_ldmrs_new_message *laser)
{
//	FILE *f = fopen("scan.txt", "a");
//	fprintf(f, "\n\n%d %lf %lf %d %f %f %d \n\n",
//			laser->scan_number,
//			laser->scan_start_time,
//			laser->scan_end_time,
//			laser->angle_ticks_per_rotation,
//			laser->start_angle,
//			laser->end_angle,
//			laser->scan_points);
//
//	for (int i = 0; i < laser->scan_points; i++)
//	{
//		fprintf(f, "index %d, layer %d, ha %f, va %f, d %f, flags %d \n", i, laser->arraypoints[i].layer,
//					carmen_radians_to_degrees(laser->arraypoints[i].horizontal_angle),
//					carmen_radians_to_degrees(laser->arraypoints[i].vertical_angle),
//					laser->arraypoints[i].radial_distance,
//					laser->arraypoints[i].flags);
//	}
//	fflush(f);
//	fclose(f);

	carmen_velodyne_partial_scan_message partial_scan_message = carmen_laser_ldmrs_new_convert_laser_scan_to_partial_velodyne_message(laser, laser->timestamp);

//	f = fopen("scan.txt", "a");
//	fprintf(f, "\n\n%d %lf %lf %d %f %f %d \n\n",
//			laser->scan_number,
//			laser->scan_start_time,
//			laser->scan_end_time,
//			laser->angle_ticks_per_rotation,
//			laser->start_angle,
//			laser->end_angle,
//			laser->scan_points);
//
//	for (int i = 0; i < partial_scan_message.number_of_32_laser_shots; i++)
//	{
//		for (int j = 0; j < 4; j++)
//			fprintf(f, "index %d, ha %lf, d %lf\n", i,
//						partial_scan_message.partial_scan[i].angle,
//						(double) partial_scan_message.partial_scan[i].distance[j] / 500.0);
//	}
//	fflush(f);
//	fclose(f);

	if (partial_scan_message.number_of_32_laser_shots > 0)
	{
		mapper_velodyne_partial_scan(1, &partial_scan_message);
		free(partial_scan_message.partial_scan);
	}
}


void
stereo_velodyne_variable_scan_message_handler1(carmen_velodyne_variable_scan_message *message)
{
	mapper_stereo_velodyne_variable_scan(1, message);
}


static void
stereo_velodyne_variable_scan_message_handler2(carmen_velodyne_variable_scan_message *message)
{
	mapper_stereo_velodyne_variable_scan(2, message);
}


static void
stereo_velodyne_variable_scan_message_handler3(carmen_velodyne_variable_scan_message *message)
{
	camera3_ready = mapper_stereo_velodyne_variable_scan(3, message);
}


static void
stereo_velodyne_variable_scan_message_handler4(carmen_velodyne_variable_scan_message *message)
{
	mapper_stereo_velodyne_variable_scan(4, message);
}


static void
stereo_velodyne_variable_scan_message_handler5(carmen_velodyne_variable_scan_message *message)
{
	mapper_stereo_velodyne_variable_scan(5, message);
}


static void
stereo_velodyne_variable_scan_message_handler6(carmen_velodyne_variable_scan_message *message)
{
	mapper_stereo_velodyne_variable_scan(6, message);
}


static void
stereo_velodyne_variable_scan_message_handler7(carmen_velodyne_variable_scan_message *message)
{
	camera7_ready = mapper_stereo_velodyne_variable_scan(7, message);
}


static void
stereo_velodyne_variable_scan_message_handler8(carmen_velodyne_variable_scan_message *message)
{
	mapper_stereo_velodyne_variable_scan(8, message);
}


static void
stereo_velodyne_variable_scan_message_handler9(carmen_velodyne_variable_scan_message *message)
{
	mapper_stereo_velodyne_variable_scan(9, message);
}

void
variable_scan_message_handler_0(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(10, message);
}

void
variable_scan_message_handler_1(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(11, message);
}

void
variable_scan_message_handler_2(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(12, message);
}

void
variable_scan_message_handler_3(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(13, message);
}

void
variable_scan_message_handler_4(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(14, message);
}

void
variable_scan_message_handler_5(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(15, message);
}

void
variable_scan_message_handler_6(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(16, message);
}

void
variable_scan_message_handler_7(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(17, message);
}

void
variable_scan_message_handler_8(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(18, message);
}

void
variable_scan_message_handler_9(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(19, message);
}

void
variable_scan_message_handler_10(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(20, message);
}

void
variable_scan_message_handler_11(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(21, message);
}

void
variable_scan_message_handler_12(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(22, message);
}

void
variable_scan_message_handler_13(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(23, message);
}

void
variable_scan_message_handler_14(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(24, message);
}

void
variable_scan_message_handler_15(carmen_velodyne_variable_scan_message *message)
{
	update_data_params_with_lidar_data(25, message);
}


static void
offline_map_handler(carmen_map_server_offline_map_message *msg)
{
	static bool first_time = true;
	carmen_position_t map_origin;

	if (first_time)
	{
		offline_map_available = true;
		first_time = false;
	}

	map_origin.x = msg->config.x_origin;
	map_origin.y = msg->config.y_origin;

	memcpy(offline_map.complete_map, msg->complete_map, msg->config.x_size * msg->config.y_size * sizeof(double));
	offline_map.config = msg->config;

	if (use_merge_between_maps)
		mapper_change_map_origin_to_another_map_block_with_clones(&map_origin, mapper_save_map);
	else
		mapper_change_map_origin_to_another_map_block(&map_origin, mapper_save_map);

	if (merge_with_offline_map)
		mapper_merge_online_map_with_offline_map(&offline_map);
}


static void
parking_sensor_goal_message_handler(carmen_parking_assistant_goal_message *message __attribute__ ((unused)))
{
	parking_assistant_found_safe_space = 1;
}


static void
ultrasonic_sensor_message_handler(carmen_ultrasonic_sonar_sensor_message *message)
{
	carmen_point_t Xt_r1, Xt_r2, Xt_l1, Xt_l2;

	tf::Transform world_to_car_pose;
	double yaw, pitch, roll;

	int i;

	carmen_localize_ackerman_globalpos_message globalpos_message;
	globalpos_message = interpolator.InterpolateMessages(message);

	mapper_set_robot_pose_into_the_map(&globalpos_message, update_cells_below_car);

	if (parking_assistant_found_safe_space)
	{
		publish_map(message->timestamp);
		return;
	}

	world_to_car_pose.setOrigin(tf::Vector3(globalpos_message.pose.position.x, globalpos_message.pose.position.y, globalpos_message.pose.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(globalpos_message.pose.orientation.yaw, globalpos_message.pose.orientation.pitch, globalpos_message.pose.orientation.roll));
	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	tf_transformer.setTransform(world_to_car_transform, "world_to_car_transform");

	//SENSOR R1 - FRONTAL
	tf::StampedTransform world_to_ultrasonic_sensor_r1;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_r1", tf::Time(0), world_to_ultrasonic_sensor_r1);
	Xt_r1.x = world_to_ultrasonic_sensor_r1.getOrigin().x();
	Xt_r1.y = world_to_ultrasonic_sensor_r1.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_r1.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_r1.theta = yaw;

	double range[180];

	for (i=0 ; i<180 ; i++)
		range[i] = (double) message->sensor[3];

	mapper_update_grid_map(Xt_r1, range, &ultrasonic_sensor_params);

	//SENSOR R2 - LATERAL FRONTAL
	tf::StampedTransform world_to_ultrasonic_sensor_r2;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_r2", tf::Time(0), world_to_ultrasonic_sensor_r2);
	Xt_r2.x = world_to_ultrasonic_sensor_r2.getOrigin().x();
	Xt_r2.y = world_to_ultrasonic_sensor_r2.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_r2.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_r2.theta = yaw;

	for (i = 0; i < 180 ;i++)
		range[i] = (double) message->sensor[2];

	mapper_update_grid_map(Xt_r2, range, &ultrasonic_sensor_params);

	//SENSOR L2 - LATERAL TRASEIRO
	tf::StampedTransform world_to_ultrasonic_sensor_l2;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_l2", tf::Time(0), world_to_ultrasonic_sensor_l2);
	Xt_l2.x = world_to_ultrasonic_sensor_l2.getOrigin().x();
	Xt_l2.y = world_to_ultrasonic_sensor_l2.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_l2.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_l2.theta = yaw;

	for (i = 0; i < 180 ;i++)
		range[i] = (double) message->sensor[1];

	mapper_update_grid_map(Xt_l2, range, &ultrasonic_sensor_params);

	//SENSOR L1 - TRASEIRO
	tf::StampedTransform world_to_ultrasonic_sensor_l1;
	tf_transformer.lookupTransform("/world", "/ultrasonic_sensor_l1", tf::Time(0), world_to_ultrasonic_sensor_l1);
	Xt_l1.x = world_to_ultrasonic_sensor_l1.getOrigin().x();
	Xt_l1.y = world_to_ultrasonic_sensor_l1.getOrigin().y();
	tf::Matrix3x3(world_to_ultrasonic_sensor_l1.getRotation()).getEulerYPR(yaw, pitch, roll);
	Xt_l1.theta = yaw;

	for (i = 0; i < 180; i++)
		range[i] = (double) message->sensor[0];

	mapper_update_grid_map(Xt_l1, range, &ultrasonic_sensor_params);

	publish_map(message->timestamp);
}


static void
rddf_annotation_message_handler(carmen_rddf_annotation_message *message)
{
	last_rddf_annotation_message = *message;
}


static void
carmen_mapper_virtual_laser_message_handler(carmen_mapper_virtual_laser_message *message)
{
	virtual_laser_message = *message;
}


void
carmen_moving_objects_point_clouds_message_handler(carmen_moving_objects_point_clouds_message *moving_objects_point_clouds_message)
{
	moving_objects_message = moving_objects_point_clouds_message;
}


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		if (update_and_merge_with_mapper_saved_maps)
			mapper_save_current_map();

		if (sensors_params[0].save_calibration_file)
			fclose(sensors_params[0].save_calibration_file);

		carmen_ipc_disconnect();
		fprintf(stderr, "Shutdown mapper_main\n");

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
carmen_subscribe_to_ultrasonic_relevant_messages()
{
	carmen_parking_assistant_subscribe_goal(NULL, (carmen_handler_t) parking_sensor_goal_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_ultrasonic_sonar_sensor_subscribe(NULL, (carmen_handler_t) ultrasonic_sensor_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
subscribe_to_ipc_messages()
{
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t)carmen_localize_ackerman_globalpos_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[0].alive)
		carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t)velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[1].alive)
	{
		carmen_laser_subscribe_ldmrs_message(NULL, (carmen_handler_t) laser_ldrms_message_handler, CARMEN_SUBSCRIBE_LATEST);
		carmen_laser_subscribe_ldmrs_new_message(NULL, (carmen_handler_t) laser_ldrms_new_message_handler, CARMEN_SUBSCRIBE_LATEST);
	}

	if (sensors_params[2].alive)
		carmen_stereo_velodyne_subscribe_scan_message(2, NULL, (carmen_handler_t)stereo_velodyne_variable_scan_message_handler2, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[3].alive)
		carmen_stereo_velodyne_subscribe_scan_message(3, NULL, (carmen_handler_t)stereo_velodyne_variable_scan_message_handler3, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[4].alive)
		carmen_stereo_velodyne_subscribe_scan_message(4, NULL, (carmen_handler_t)stereo_velodyne_variable_scan_message_handler4, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[5].alive)
		carmen_stereo_velodyne_subscribe_scan_message(5, NULL, (carmen_handler_t)stereo_velodyne_variable_scan_message_handler5, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[6].alive)
		carmen_stereo_velodyne_subscribe_scan_message(6, NULL, (carmen_handler_t)stereo_velodyne_variable_scan_message_handler6, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[7].alive)
		carmen_stereo_velodyne_subscribe_scan_message(7, NULL, (carmen_handler_t)stereo_velodyne_variable_scan_message_handler7, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[8].alive)
		carmen_stereo_velodyne_subscribe_scan_message(8, NULL, (carmen_handler_t)stereo_velodyne_variable_scan_message_handler8, CARMEN_SUBSCRIBE_LATEST);

	if (sensors_params[9].alive)
		carmen_stereo_velodyne_subscribe_scan_message(9, NULL, (carmen_handler_t)stereo_velodyne_variable_scan_message_handler9, CARMEN_SUBSCRIBE_LATEST);

	//-------------------------------------------------------------------------------------------------------------------------------------//
	// LIDARS
	if (sensors_params[10].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_0, CARMEN_SUBSCRIBE_LATEST, 0);

	if (sensors_params[11].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_1, CARMEN_SUBSCRIBE_LATEST, 1);

	if (sensors_params[12].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_2, CARMEN_SUBSCRIBE_LATEST, 2);

	if (sensors_params[13].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_3, CARMEN_SUBSCRIBE_LATEST, 3);

	if (sensors_params[14].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_4, CARMEN_SUBSCRIBE_LATEST, 4);

	if (sensors_params[15].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_5, CARMEN_SUBSCRIBE_LATEST, 5);

	if (sensors_params[16].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_6, CARMEN_SUBSCRIBE_LATEST, 6);

	if (sensors_params[17].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_7, CARMEN_SUBSCRIBE_LATEST, 7);

	if (sensors_params[18].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_8, CARMEN_SUBSCRIBE_LATEST, 8);

	if (sensors_params[19].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_9, CARMEN_SUBSCRIBE_LATEST, 9);

	if (sensors_params[20].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_10, CARMEN_SUBSCRIBE_LATEST, 10);

	if (sensors_params[21].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_11, CARMEN_SUBSCRIBE_LATEST, 11);

	if (sensors_params[22].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_12, CARMEN_SUBSCRIBE_LATEST, 12);

	if (sensors_params[23].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_13, CARMEN_SUBSCRIBE_LATEST, 13);

	if (sensors_params[24].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_14, CARMEN_SUBSCRIBE_LATEST, 14);

	if (sensors_params[25].alive)
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t)variable_scan_message_handler_15, CARMEN_SUBSCRIBE_LATEST, 15);

	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) offline_map_handler, CARMEN_SUBSCRIBE_LATEST);

	if (!use_truepos) // This flag is for a special kind of operation where the sensor pipeline listen to the globalpos and the planning pipeline to the truepos
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) true_pos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	if (visual_odometry_is_global_pos)
		carmen_subscribe_to_ultrasonic_relevant_messages();

	carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t) rddf_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_mapper_subscribe_virtual_laser_message(NULL, (carmen_handler_t) carmen_mapper_virtual_laser_message_handler, CARMEN_SUBSCRIBE_LATEST);

	// draw moving objects
	if (use_unity_simulator)
		carmen_moving_objects_point_clouds_subscribe_message(NULL, (carmen_handler_t) carmen_moving_objects_point_clouds_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
initialize_carmen_mapper_probability_of_each_ray_of_lidar_hit_obstacle_messages()
{
	for(int i = 0; i < MAX_NUMBER_OF_LIDARS; i++)
	{
		if (sensors_params[i + 10].alive)  // Lidars start from 10 in the sensors_params vector
			initialize_carmen_mapper_probability_of_each_ray_of_lidar_hit_obstacle_message(&probability_of_each_ray_msg_array[i]);
	}
}


static void
define_mapper_messages()
{
	carmen_map_server_define_compact_cost_map_message();
	carmen_mapper_define_messages();
	carmen_mapper_define_virtual_scan_message();
	carmen_mapper_define_probability_of_each_ray_of_lidar_hit_obstacle_messages();
}


//////////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char **argv)
{
	carmen_map_config_t map_config;
	carmen_robot_ackerman_config_t car_config;

	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Initialize all the relevant parameters */
	carmen_mapper_read_parameters(argc, argv, &map_config, &car_config);

	carmen_mapper_get_highest_sensor();

	carmen_mapper_initialize_transforms();

	mapper_initialize(&map_config, car_config, use_merge_between_maps);

	initialize_carmen_mapper_probability_of_each_ray_of_lidar_hit_obstacle_messages();

	if (use_neural_mapper)
		initialize_inference_context_mapper_();

	/* Register messages */
	define_mapper_messages();

	/* Subscribe to relevant messages */
	subscribe_to_ipc_messages();

	carmen_ipc_dispatch();

	return (0);
}
